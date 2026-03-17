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
                            ret, jpeg = cv2.imencode('.jpg', img)
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
            # Simple landing page
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b"<html><head><title>Tello Camera Server</title></head>")
            self.wfile.write(b"<body><h1>Tello Camera Server</h1>")
            self.wfile.write(b"<p>MJPEG Stream: <a href='/video'>/video</a></p>")
            self.wfile.write(b"<img src='/video' width='640' height='480'>")
            self.wfile.write(b"</body></html>")

def main():
    global frame_read, tello
    
    print("Connecting to Tello...")
    tello = Tello()
    try:
        tello.connect()
    except Exception as e:
        print(f"Error: Failed to connect to Tello. Make sure you are connected to the Tello's WiFi. Details: {e}")
        sys.exit(1)

    print(f"Battery: {tello.get_battery()}%")
    
    print("Starting video stream...")
    tello.streamon()
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
