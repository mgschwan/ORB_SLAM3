#!/usr/bin/env python3
"""
replay_frames.py

Replays a recording created by record_frames.py, sending each saved JPEG
frame back to the localization service — optionally including the recorded
pose as an externally provided position (the forced-pose path in ORB-SLAM3).

The service accepts poses in Twc convention (camera position in world frame),
which is exactly the format stored in the manifest by record_frames.py.

Usage
-----
    python3 replay_frames.py RECORDING_DIR [options]

Examples
--------
    # Replay without poses — let ORB-SLAM3 compute them from scratch
    python3 replay_frames.py ./recording_20241215_143022

    # Replay with recorded poses (guided / forced-pose mapping)
    python3 replay_frames.py ./recording_20241215_143022 --with-pose

    # Replay only frames that had a valid pose during recording
    python3 replay_frames.py ./recording_20241215_143022 --with-pose --only-valid-poses

    # Replay at half the original speed against a remote server
    python3 replay_frames.py ./recording_20241215_143022 --fps 15 \\
        --server http://192.168.1.10:11142

    # Replay as fast as the service can accept (no rate limiting)
    python3 replay_frames.py ./recording_20241215_143022 --fps 0

    # Loop continuously, show a preview window
    python3 replay_frames.py ./recording_20241215_143022 --loop --show

Dependencies
------------
    pip install requests
    pip install opencv-python   # only required for --show
"""

import argparse
import json
import signal
import sys
import time
from pathlib import Path

import requests


DEFAULT_SERVER = "http://localhost:11142"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Replay a recording to the localization service",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "recording", metavar="RECORDING_DIR",
        help="Directory created by record_frames.py",
    )
    p.add_argument(
        "--server", default=None,
        help=f"Override service base URL "
             f"(default: from manifest, or {DEFAULT_SERVER})",
    )
    p.add_argument(
        "--fps", type=float, default=None,
        help="Override replay rate in fps "
             "(default: from manifest; 0 = unlimited)",
    )
    p.add_argument(
        "--with-pose", action="store_true",
        help="Include the recorded pose as an externally provided position",
    )
    p.add_argument(
        "--only-valid-poses", action="store_true",
        help="With --with-pose: skip frames whose recorded pose is not valid",
    )
    p.add_argument(
        "--loop", action="store_true",
        help="Loop the recording indefinitely (Ctrl+C to stop)",
    )
    p.add_argument(
        "--show", action="store_true",
        help="Display each frame in a preview window (press q to quit)",
    )
    return p.parse_args()


def format_pose(data: dict) -> str:
    state = data.get("tracking_state", "?")
    pose  = data.get("pose", {})
    if pose.get("valid"):
        return (
            f"[{state}]  "
            f"x={pose['x']:+.3f}  y={pose['y']:+.3f}  z={pose['z']:+.3f}"
        )
    return f"[{state}]  (no valid pose)"


def send_with_retry(
    session: requests.Session,
    frame_url: str,
    jpeg_bytes: bytes,
    params: dict,
    max_retries: int = 8,
) -> dict | None:
    """Submit a frame, retrying on 503 (queue full) with linear back-off."""
    for attempt in range(max_retries):
        try:
            resp = session.post(
                frame_url,
                data=jpeg_bytes,
                params=params,
                headers={"Content-Type": "image/jpeg"},
                timeout=1.0,
            )
            if resp.status_code == 200:
                return resp.json()
            if resp.status_code == 503:
                time.sleep(0.05 * (attempt + 1))
                continue
            print(f"\n[warn] HTTP {resp.status_code}: {resp.text[:120]}")
            return None
        except requests.exceptions.Timeout:
            pass
        except requests.exceptions.ConnectionError as exc:
            print(f"\n[error] Connection lost: {exc}")
            return None
    return None


def main() -> None:
    args = parse_args()

    recording_dir = Path(args.recording)
    manifest_path = recording_dir / "manifest.json"

    if not manifest_path.exists():
        print(
            f"ERROR: manifest.json not found in {recording_dir}",
            file=sys.stderr,
        )
        sys.exit(1)

    with open(manifest_path) as f:
        manifest = json.load(f)

    frames        = manifest.get("frames", [])
    server        = args.server or manifest.get("server", DEFAULT_SERVER)
    fps           = args.fps if args.fps is not None else manifest.get("fps", 30)
    frame_url     = f"{server}/api/frame"
    frame_interval = (1.0 / fps) if fps > 0 else 0.0

    if not frames:
        print("ERROR: manifest contains no frames.", file=sys.stderr)
        sys.exit(1)

    # Optional cv2 import for --show
    cv2 = None
    np  = None
    if args.show:
        try:
            import cv2 as _cv2
            import numpy as _np
            cv2 = _cv2
            np  = _np
        except ImportError:
            print("[warn] opencv-python not installed — --show ignored")

    running = True

    def on_sigint(_sig, _frame):
        nonlocal running
        print("\nStopping...")
        running = False

    signal.signal(signal.SIGINT, on_sigint)

    session        = requests.Session()
    t_start        = time.monotonic()
    frames_sent    = 0
    frames_skipped = 0
    total          = len(frames)

    print(f"Recording      : {recording_dir}")
    print(f"Frames         : {total}")
    print(f"Server         : {frame_url}")
    print(f"Rate           : {'unlimited' if fps == 0 else f'{fps:.1f} fps'}")
    print(f"With pose      : {args.with_pose}")
    if args.with_pose:
        print(f"Only valid poses: {args.only_valid_poses}")
    print(f"Loop           : {args.loop}")
    print("Press Ctrl+C to stop.\n")

    while running:
        for entry in frames:
            if not running:
                break

            t_loop     = time.monotonic()
            frame_path = recording_dir / entry["file"]

            if not frame_path.exists():
                print(f"\n[warn] Missing file: {frame_path}")
                frames_skipped += 1
                continue

            pose = entry.get("pose", {"valid": False})

            if args.with_pose and args.only_valid_poses and not pose.get("valid"):
                frames_skipped += 1
                continue

            jpeg_bytes = frame_path.read_bytes()

            params: dict = {"ts": f"{entry['ts_ms']:.3f}"}
            if args.with_pose and pose.get("valid"):
                params.update({
                    "tx": pose["x"],
                    "ty": pose["y"],
                    "tz": pose["z"],
                    "qx": pose["qx"],
                    "qy": pose["qy"],
                    "qz": pose["qz"],
                    "qw": pose["qw"],
                })

            data = send_with_retry(session, frame_url, jpeg_bytes, params)
            if data is None:
                running = False
                break

            frames_sent += 1
            idx = entry.get("index", frames_sent)
            print(
                f"\r  #{idx:06d}/{total}  {format_pose(data)}    ",
                end="", flush=True,
            )

            if cv2 is not None and np is not None:
                buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img is not None:
                    cv2.imshow("replay_frames", img)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        running = False
                        break

            if frame_interval > 0:
                sleep_for = frame_interval - (time.monotonic() - t_loop)
                if sleep_for > 0:
                    time.sleep(sleep_for)

        if not args.loop:
            break

    if cv2 is not None:
        cv2.destroyAllWindows()

    elapsed = max(time.monotonic() - t_start, 1e-3)
    print(
        f"\nDone.  {frames_sent} frames sent, {frames_skipped} skipped"
        f" over {elapsed:.1f}s  ({frames_sent / elapsed:.1f} fps effective)"
    )


if __name__ == "__main__":
    main()
