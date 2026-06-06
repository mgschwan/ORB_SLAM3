import socket
import struct
import sys
import threading
import queue
import cv2
import numpy as np
import time

# Constants matching the Arduino code
UDP_PORT  = 11211
TCP_PORT  = 11212
HEADER_FORMAT = '<BII'  # uint8_t packet_type, uint32_t frame_time, uint32_t total_size
HEADER_SIZE   = struct.calcsize(HEADER_FORMAT)

PACKET_TYPE_IMAGE   = 0x01
PACKET_TYPE_IMU     = 0x02
PACKET_TYPE_TRIGGER = 0x03

SINGLE_IMU_FRAME_FORMAT = '<ffffff'  # roll, pitch, yaw, vx, vy, vz
SINGLE_IMU_FRAME_SIZE   = struct.calcsize(SINGLE_IMU_FRAME_FORMAT)


def listen_for_server_ip():
    print(f"Listening for UDP broadcast on port {UDP_PORT}...")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', UDP_PORT))
        while True:
            try:
                data, addr = s.recvfrom(1024)
                ip = data.decode('utf-8').strip()
                print(f"Discovered ESP32 at {ip} (broadcast from {addr})")
                return ip
            except Exception as e:
                print(f"UDP receive error: {e}")
                time.sleep(1)


def receive_all(sock, n):
    """Block until exactly n bytes are received. Returns None on connection close."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def rx_thread_fn(sock, pkt_queue, stop_event):
    """
    Dedicated receive thread. Uses a long socket timeout so large JPEG payloads
    never race with the trigger timer. Pushes (packet_type, frame_time, payload)
    tuples onto pkt_queue, or None on disconnect/error.
    """
    sock.settimeout(5.0)
    while not stop_event.is_set():
        try:
            header = receive_all(sock, HEADER_SIZE)
            if header is None:
                break
            ptype, ftime, total_size = struct.unpack(HEADER_FORMAT, header)
            payload = receive_all(sock, total_size) if total_size > 0 else b""
            if payload is None:
                break
            pkt_queue.put((ptype, ftime, payload))
        except socket.timeout:
            continue  # no data yet, just loop
        except OSError:
            break
    pkt_queue.put(None)  # sentinel: connection gone


def main(fps=5):
    trigger_interval = 1.0 / fps

    server_ip = listen_for_server_ip()
    print(f"Connecting to {server_ip}:{TCP_PORT}  (trigger rate: {fps} fps)")

    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((server_ip, TCP_PORT))
            sock.settimeout(None)  # rx thread sets its own timeout
            print("Connected.")
        except socket.error as e:
            print(f"Connect failed: {e}. Retrying in 2s...")
            time.sleep(2)
            continue

        pkt_queue  = queue.Queue()
        stop_event = threading.Event()
        rx = threading.Thread(target=rx_thread_fn,
                              args=(sock, pkt_queue, stop_event),
                              daemon=True)
        rx.start()

        last_trigger = 0.0
        try:
            while True:
                # --- Send trigger on schedule (main thread owns TX) ---
                now = time.monotonic()
                if now - last_trigger >= trigger_interval:
                    try:
                        sock.sendall(bytes([PACKET_TYPE_TRIGGER]))
                        last_trigger = now
                    except OSError:
                        break

                # --- Drain available packets without blocking the trigger timer ---
                try:
                    item = pkt_queue.get(timeout=0.005)
                except queue.Empty:
                    continue

                if item is None:
                    print("Connection closed by device.")
                    break

                ptype, ftime, payload = item

                if ptype == PACKET_TYPE_IMAGE:
                    np_arr = np.frombuffer(payload, np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        print ("Frame size ",str(img.shape))
                        cv2.imshow('ESP32 Camera', img)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            return
                    else:
                        print(f"Image decode failed (payload {len(payload)} bytes)")

                elif ptype == PACKET_TYPE_IMU:
                    n_frames = len(payload) // SINGLE_IMU_FRAME_SIZE
                    print(f"IMU @{ftime}ms — {n_frames} frames")
                    for i in range(n_frames):
                        chunk = payload[i * SINGLE_IMU_FRAME_SIZE:(i + 1) * SINGLE_IMU_FRAME_SIZE]
                        r, p, y, vx, vy, vz = struct.unpack(SINGLE_IMU_FRAME_FORMAT, chunk)
                        print(f"  [{i:3d}] R={r:.3f} P={p:.3f} Y={y:.3f}  "
                              f"Vx={vx:.1f} Vy={vy:.1f} Vz={vz:.1f} mm/s")
                else:
                    print(f"Unknown packet type: {ptype:#04x}")

        except KeyboardInterrupt:
            return
        finally:
            stop_event.set()
            sock.close()
            rx.join(timeout=2)

        print("Reconnecting in 1s...")
        time.sleep(1)


if __name__ == "__main__":
    # Usage:
    #   python test_tcp_hud.py        → default 5 fps trigger rate
    #   python test_tcp_hud.py 10     → 10 fps
    fps = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    try:
        main(fps=fps)
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        cv2.destroyAllWindows()
