#!/usr/bin/env python3
"""
record_frames.py

Drop-in recording server for the localization service.

Listens on the same port as the real service (default: 11142) and records
every POST /api/frame request — the JPEG body and all query parameters
(timestamp, externally-provided pose) — to a directory for later replay with
replay_frames.py.

Two HTTP server modes
---------------------
Standalone (default)
    Records incoming frames and returns a minimal OK response.  Run this
    *instead* of the real localization service when you only want to capture
    a client session without running SLAM.

Proxy (--forward URL)
    Forwards every request to the real service at URL and relays its response
    back to the client.  Run this *alongside* the real service (started on a
    different port) when you want to record a live SLAM session and preserve
    the SLAM-computed poses too.

Device / stream capture mode (--source)
----------------------------------------
    Pass a V4L2 device path (/dev/video0), an integer index (0), or any
    stream URL accepted by OpenCV (rtsp://, http://, etc.).  In this mode no
    HTTP server is started; frames are read directly from the device at a
    fixed FPS (--fps, default 30), shown in a preview window, and saved with
    zero-pose manifest entries so replay_frames.py can replay them.

    Press 'q' in the preview window or Ctrl+C in the terminal to stop.

Output layout
-------------
    <output_dir>/
        frames/
            000001.jpg
            000002.jpg
            ...
        manifest.json

manifest.json stores per-frame: file path, timestamp, and pose.  In HTTP
server modes the pose is taken from the client request parameters; in device
mode every entry has a zero translation / identity quaternion so that
replay_frames.py can still iterate the recording.

Usage
-----
    python3 record_frames.py [options]

Examples
--------
    # Standalone HTTP server on port 11142
    python3 record_frames.py

    # Proxy HTTP server forwarding to real service on port 11143
    python3 record_frames.py --forward http://localhost:11143

    # Record from /dev/video0 at 15 fps, no preview
    python3 record_frames.py --source /dev/video0 --fps 15 --no-preview

    # Record from device index 0 at default 30 fps with preview
    python3 record_frames.py --source 0

    # Custom output directory
    python3 record_frames.py --source /dev/video0 --output my_session

Dependencies
------------
    pip install requests   # only required for --forward proxy mode
    pip install opencv-python  # required for --source device/stream mode
"""

import argparse
import json
import os
import signal
import sys
import time
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path


DEFAULT_PORT = 11142
DEFAULT_FPS  = 30

_DUMMY_RESPONSE = json.dumps({
    "queued":         True,
    "tracking_state": "OK",
    "pose": {
        "valid": False,
        "x": 0.0, "y": 0.0, "z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
    },
}).encode()

_ZERO_POSE = {
    "valid": False,
    "x": 0.0, "y": 0.0, "z": 0.0,
    "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Recording server / device capture for localization frames",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help=f"Port to listen on in HTTP server mode (default: {DEFAULT_PORT})",
    )
    p.add_argument(
        "--forward", metavar="URL", default=None,
        help="Forward requests to this URL (proxy mode); "
             "e.g. http://localhost:11143",
    )
    p.add_argument(
        "--source", metavar="DEVICE_OR_URL", default=None,
        help="V4L2 device (/dev/video0), integer index (0), or stream URL. "
             "Activates device-capture mode (no HTTP server).",
    )
    p.add_argument(
        "--fps", type=float, default=DEFAULT_FPS,
        help=f"Target capture FPS in device mode (default: {DEFAULT_FPS})",
    )
    p.add_argument(
        "--no-preview", action="store_true",
        help="Disable the OpenCV preview window in device mode",
    )
    p.add_argument(
        "--output", default=None,
        help="Output directory (default: recording_YYYYMMDD_HHMMSS)",
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Shared recorder state (written by handler threads / device loop, read at shutdown)
# ---------------------------------------------------------------------------
class Recorder:
    def __init__(self, output_dir: Path, forward_url: str | None):
        self.output_dir  = output_dir
        self.frames_dir  = output_dir / "frames"
        self.forward_url = forward_url
        self.frames_dir.mkdir(parents=True, exist_ok=True)

        self._lock      = threading.Lock()
        self._index     = 0
        self.manifest   = {
            "forward": forward_url,
            "frames":  [],
        }

        # lazy import requests only when forwarding
        self._requests = None
        if forward_url:
            try:
                import requests as _r
                self._requests = _r
            except ImportError:
                print(
                    "ERROR: --forward requires the 'requests' package.\n"
                    "       pip install requests",
                    file=sys.stderr,
                )
                sys.exit(1)

    def record(self, jpeg_bytes: bytes, params: dict) -> bytes:
        """
        Save the frame, update the manifest, and return the response body to
        send back to the client (either from the forwarded service or dummy).
        """
        with self._lock:
            self._index += 1
            idx = self._index

        filename  = f"{idx:06d}.jpg"
        rel_path  = f"frames/{filename}"
        (self.frames_dir / filename).write_bytes(jpeg_bytes)

        ts_ms = float(params.get("ts", 0.0))

        # Pose provided by the client in the request query params
        has_pose = all(k in params for k in ("tx", "ty", "tz", "qx", "qy", "qz", "qw"))
        provided_pose: dict
        if has_pose:
            provided_pose = {
                "valid": True,
                "x":  float(params["tx"]),
                "y":  float(params["ty"]),
                "z":  float(params["tz"]),
                "qx": float(params["qx"]),
                "qy": float(params["qy"]),
                "qz": float(params["qz"]),
                "qw": float(params["qw"]),
            }
        else:
            provided_pose = dict(_ZERO_POSE)

        # Forward or return dummy response
        response_body  = _DUMMY_RESPONSE
        slam_pose      = None
        tracking_state = "RECORDING"

        if self._requests and self.forward_url:
            try:
                r = self._requests.post(
                    f"{self.forward_url}/api/frame",
                    data=jpeg_bytes,
                    params=params,
                    headers={"Content-Type": "image/jpeg"},
                    timeout=1.0,
                )
                response_body = r.content
                if r.status_code == 200:
                    data           = r.json()
                    tracking_state = data.get("tracking_state", "UNKNOWN")
                    slam_pose      = data.get("pose")
            except Exception as exc:
                print(f"\n[proxy error] {exc}")

        entry: dict = {
            "index":          idx,
            "file":           rel_path,
            "ts_ms":          ts_ms,
            "tracking_state": tracking_state,
            "pose":           provided_pose,
        }
        if slam_pose is not None:
            entry["slam_pose"] = slam_pose

        with self._lock:
            self.manifest["frames"].append(entry)

        pos_str = (
            f"x={provided_pose['x']:+.3f} y={provided_pose['y']:+.3f} "
            f"z={provided_pose['z']:+.3f}"
            if provided_pose["valid"] else "no pose"
        )
        print(
            f"\r  #{idx:06d}  [{tracking_state}]  {pos_str}    ",
            end="", flush=True,
        )

        return response_body

    def record_device_frame(self, jpeg_bytes: bytes, ts_ms: float) -> None:
        """Save one frame captured from a device with a zero/identity pose."""
        with self._lock:
            self._index += 1
            idx = self._index

        filename = f"{idx:06d}.jpg"
        rel_path = f"frames/{filename}"
        (self.frames_dir / filename).write_bytes(jpeg_bytes)

        entry = {
            "index":          idx,
            "file":           rel_path,
            "ts_ms":          ts_ms,
            "tracking_state": "RECORDING",
            "pose":           dict(_ZERO_POSE),
        }

        with self._lock:
            self.manifest["frames"].append(entry)

        print(f"\r  #{idx:06d}  [DEVICE]  ts={ts_ms:.0f} ms    ", end="", flush=True)

    def save_manifest(self):
        frames = self.manifest["frames"]
        if len(frames) >= 2:
            dt = (frames[-1]["ts_ms"] - frames[0]["ts_ms"]) / 1000.0
            self.manifest["fps"] = round(len(frames) / max(dt, 1e-3), 2)
        path = self.output_dir / "manifest.json"
        with open(path, "w") as f:
            json.dump(self.manifest, f, indent=2)
        return path


# ---------------------------------------------------------------------------
# HTTP request handler
# ---------------------------------------------------------------------------
def make_handler(recorder: Recorder):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # suppress default access log

        def do_POST(self):
            parsed = urllib.parse.urlparse(self.path)
            if parsed.path != "/api/frame":
                self.send_response(404)
                self.end_headers()
                return

            params_raw = urllib.parse.parse_qs(parsed.query)
            params = {k: v[0] for k, v in params_raw.items()}

            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length) if length > 0 else b""

            if not body:
                self._send_json(200, _DUMMY_RESPONSE)
                return

            response_body = recorder.record(body, params)
            self._send_json(200, response_body)

        def _send_json(self, code: int, body: bytes):
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


# ---------------------------------------------------------------------------
# Device / stream capture loop
# ---------------------------------------------------------------------------
def run_device_capture(
    source: str,
    recorder: Recorder,
    target_fps: float,
    show_preview: bool,
) -> None:
    try:
        import cv2
    except ImportError:
        print(
            "ERROR: --source requires opencv-python.\n"
            "       pip install opencv-python",
            file=sys.stderr,
        )
        sys.exit(1)

    # Accept integer index or path/URL
    cap_source: int | str
    try:
        cap_source = int(source)
    except ValueError:
        cap_source = source

    cap = cv2.VideoCapture(cap_source)
    if not cap.isOpened():
        print(f"ERROR: cannot open capture source: {source}", file=sys.stderr)
        sys.exit(1)

    # Try to configure device FPS (best-effort; streams may ignore it)
    cap.set(cv2.CAP_PROP_FPS, target_fps)

    frame_interval = 1.0 / target_fps
    window_name    = "record_frames — press q to stop"
    start_time     = time.monotonic()
    stop_event     = threading.Event()

    def on_sigint(_sig, _frame):
        print("\nStopping capture...")
        stop_event.set()

    signal.signal(signal.SIGINT, on_sigint)

    print(f"Source    : {source}")
    print(f"Target FPS: {target_fps}")
    print(f"Preview   : {'yes' if show_preview else 'no'}")
    print("Press 'q' in the preview window or Ctrl+C to stop.\n")

    next_frame_time = time.monotonic()

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            print("\nCapture source ended or error reading frame.")
            break

        ts_ms = (time.monotonic() - start_time) * 1000.0

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            recorder.record_device_frame(bytes(buf), ts_ms)

        if show_preview:
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        # Fixed-FPS pacing: sleep until the next scheduled frame time
        next_frame_time += frame_interval
        sleep_for = next_frame_time - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            # We're behind; reset schedule to now so we don't spiral
            next_frame_time = time.monotonic()

    cap.release()
    if show_preview:
        cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    args = parse_args()

    from datetime import datetime
    output_dir = Path(
        args.output or f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )

    recorder = Recorder(output_dir, args.forward if not args.source else None)

    if args.source:
        # Device / stream capture mode — no HTTP server
        print(f"Mode      : device capture")
        print(f"Output    : {output_dir}/")
        run_device_capture(
            source=args.source,
            recorder=recorder,
            target_fps=args.fps,
            show_preview=not args.no_preview,
        )
    else:
        # HTTP server mode (standalone or proxy)
        handler = make_handler(recorder)
        server  = HTTPServer(("", args.port), handler)

        mode = f"proxy → {args.forward}" if args.forward else "standalone"
        print(f"Mode      : {mode}")
        print(f"Listening : http://0.0.0.0:{args.port}/api/frame")
        print(f"Output    : {output_dir}/")
        print("Press Ctrl+C to stop.\n")

        def on_sigint(_sig, _frame):
            print("\nShutting down...")
            threading.Thread(target=server.shutdown, daemon=True).start()

        signal.signal(signal.SIGINT, on_sigint)
        server.serve_forever()

    manifest_path = recorder.save_manifest()
    n = recorder._index
    print(f"\nRecorded {n} frames.")
    print(f"Manifest : {manifest_path}")


if __name__ == "__main__":
    main()
