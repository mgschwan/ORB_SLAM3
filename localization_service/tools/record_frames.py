#!/usr/bin/env python3
"""
record_frames.py

Drop-in recording server for the localization service.

Listens on the same port as the real service (default: 11142) and records
every POST /api/frame request — the JPEG body and all query parameters
(timestamp, externally-provided pose) — to a directory for later replay with
replay_frames.py.

Two modes
---------
Standalone (default)
    Records incoming frames and returns a minimal OK response.  Run this
    *instead* of the real localization service when you only want to capture
    a client session without running SLAM.

Proxy (--forward URL)
    Forwards every request to the real service at URL and relays its response
    back to the client.  Run this *alongside* the real service (started on a
    different port) when you want to record a live SLAM session and preserve
    the SLAM-computed poses too.

Output layout
-------------
    <output_dir>/
        frames/
            000001.jpg
            000002.jpg
            ...
        manifest.json

manifest.json stores per-frame: file path, timestamp, the pose *provided by
the client* in the request (tx/ty/tz/qx/qy/qz/qw params), and — in proxy
mode — the SLAM-computed response pose.  replay_frames.py's --with-pose flag
re-sends the client-provided pose back to the service.

Usage
-----
    python3 record_frames.py [options]

Examples
--------
    # Standalone: record on port 11142 (client sends here instead of real service)
    python3 record_frames.py

    # Proxy: forward to real service on port 11143, record everything
    python3 record_frames.py --forward http://localhost:11143

    # Custom port and output directory
    python3 record_frames.py --port 11142 --output my_session

Dependencies
------------
    pip install requests   # only required for --forward proxy mode
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
_DUMMY_RESPONSE = json.dumps({
    "queued":         True,
    "tracking_state": "OK",
    "pose": {
        "valid": False,
        "x": 0.0, "y": 0.0, "z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
    },
}).encode()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Recording server for POST /api/frame requests",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help=f"Port to listen on (default: {DEFAULT_PORT})",
    )
    p.add_argument(
        "--forward", metavar="URL", default=None,
        help="Forward requests to this URL (proxy mode); "
             "e.g. http://localhost:11143",
    )
    p.add_argument(
        "--output", default=None,
        help="Output directory (default: recording_YYYYMMDD_HHMMSS)",
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Shared recorder state (written by handler threads, read at shutdown)
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
            provided_pose = {"valid": False, "x": 0, "y": 0, "z": 0,
                             "qx": 0, "qy": 0, "qz": 0, "qw": 1}

        # Forward or return dummy response
        response_body = _DUMMY_RESPONSE
        slam_pose     = None
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
            # 'pose' = client-provided pose; replay_frames.py --with-pose uses this
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
            pass  # suppress default access log — we print our own summary

        def do_POST(self):
            parsed = urllib.parse.urlparse(self.path)
            if parsed.path != "/api/frame":
                self.send_response(404)
                self.end_headers()
                return

            params_raw = urllib.parse.parse_qs(parsed.query)
            params = {k: v[0] for k, v in params_raw.items()}

            length     = int(self.headers.get("Content-Length", 0))
            body       = self.rfile.read(length) if length > 0 else b""

            if not body:
                # Empty-body pose poll — return dummy without recording
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
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    args = parse_args()

    from datetime import datetime
    output_dir = Path(
        args.output or f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )

    recorder = Recorder(output_dir, args.forward)
    handler  = make_handler(recorder)
    server   = HTTPServer(("", args.port), handler)

    mode = f"proxy → {args.forward}" if args.forward else "standalone"
    print(f"Recorder  : {mode}")
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
