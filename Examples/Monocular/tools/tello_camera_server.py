import cv2
from djitellopy import Tello
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import time
import threading
import sys

# Global variables for Tello and frame reader
tello = None
frame_read = None

# Global telemetry
telemetry = {"battery": 0, "temp": 0, "height": 0}

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""
    daemon_threads = True

class TelloCamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/video' or self.path == '/video.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            print(f"Client connected: {self.client_address}")
            
            try:
                while True:
                    if frame_read is not None:
                        img = frame_read.frame
                        if img is not None:
                            # The frame is in BGR (OpenCV default)
                            # Encode as JPEG
                            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert to RGB for better compatibility
                            ret, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 95])
                            if ret:
                                frame_data = jpeg.tobytes()
                                self.wfile.write(b'--frame\r\n')
                                self.send_header('Content-type', 'image/jpeg')
                                self.send_header('Content-length', str(len(frame_data)))
                                self.end_headers()
                                self.wfile.write(frame_data)
                                self.wfile.write(b'\r\n')
                    
                    # Control frame rate (approx 15fps)
                    time.sleep(0.066)
            except Exception as e:
                print(f"Client disconnected: {self.client_address}")
        else:
            # Simple landing page with telemetry
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b"<html><head><title>Tello Camera Server</title>")
            self.wfile.write(b"<meta http-equiv='refresh' content='2'>")
            self.wfile.write(b"<style>body { font-family: sans-serif; background-color: #111; color: #eee; padding: 20px; }")
            self.wfile.write(b".status-item { margin-right: 20px; font-weight: bold; }")
            self.wfile.write(b"h1 { color: #007bff; }</style></head>")
            self.wfile.write(b"<body><h1>Tello Camera Server</h1>")
            
            status_line = (f"<p><span class='status-item'>Battery:</span> {telemetry['battery']}% | "
                          f"<span class='status-item'>Temp:</span> {telemetry['temp']}&deg;C | "
                          f"<span class='status-item'>Height:</span> {telemetry['height']}cm</p>").encode()
            self.wfile.write(status_line)
            
            self.wfile.write(b"<p>MJPEG Stream: <a href='/video' style='color: #007bff;'>/video</a></p>")
            self.wfile.write(b"<img src='/video' width='640' height='480' style='border: 2px solid #333; border-radius: 8px;'>")
            self.wfile.write(b"</body></html>")

def telemetry_worker():
    """Background worker to update telemetry values periodically."""
    global telemetry
    while True:
        if tello:
            try:
                telemetry["battery"] = tello.get_battery()
                telemetry["temp"] = tello.get_temperature()
                telemetry["height"] = tello.get_height()
            except Exception:
                pass
        time.sleep(2.0)

def main():
    global frame_read, tello
    
    print("Connecting to Tello...")
    tello = Tello()
    try:
        tello.connect()
    except Exception as e:
        print(f"Error: Failed to connect to Tello. Make sure you are connected to the Tello's WiFi. Details: {e}")
        sys.exit(1)

    print(f"Initial Battery: {tello.get_battery()}%")
    
    # Start telemetry worker thread
    t = threading.Thread(target=telemetry_worker, daemon=True)
    t.start()


    print("Starting video stream...")
    tello.streamon()

    tello.set_video_fps(Tello.FPS_5) 
    #tello.set_video_bitrate(Tello.BITRATE_3MBPS)

    frame_read = tello.get_frame_read()


    # Wait a bit for the first frame to be available
    print("Waiting for camera to warm up...")
    time.sleep(2.0)
    
    port = 5000
    server_address = ('0.0.0.0', port)
    httpd = ThreadedHTTPServer(server_address, TelloCamHandler)
    
    print(f"\n" + "="*50)
    print(f"Tello Camera Server started!")
    print(f"Stream URL: http://localhost:{port}/video")
    print(f"Example usage for remote_tum:")
    print(f"./remote_tum Vocabulary/ORBvoc.txt Examples/Monocular/remote_cam_load.yaml http://localhost:{port}/video localize")
    print("="*50 + "\n")
    print("Press Ctrl+C to stop the server.")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        httpd.socket.close()
        tello.streamoff()
        print("Done.")

if __name__ == '__main__':
    main()
