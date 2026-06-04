# orblsammer_espnode

ESP32-S3 sensor node that streams camera frames and IMU data over WiFi to a host PC, intended as the input device for the ORB-SLAM3 localization pipeline.

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | Freenove ESP32-S3 WROOM (has PSRAM) |
| Camera | OV5640 via CAMERA_MODEL_ESP32S3_EYE pin mapping |
| IMU | MPU-6050 on I2C (SDA=GPIO21, SCL=GPIO20, 400 kHz) |
| Storage | SD/MMC (CLK=39, CMD=38, D0=40) — optional, holds WiFi config |

## Firmware: `src/xostudio_hud.ino`

Built with PlatformIO (`platformio.ini`, target `freenove_esp32_s3_wroom`).

### Setup sequence (`setup()`)

1. I2C + MPU-6050 initialised; `imu.setBias()` run after a 5-second still period.
2. SD card mounted (1-bit MMC mode). If `/config.txt` is present its first two lines are read as SSID and password; otherwise the compile-time `ssid`/`password` defaults are used.
3. Camera initialised at QVGA/JPEG quality 12, PSRAM frame buffer, `CAMERA_GRAB_LATEST` when PSRAM is present.
4. Camera sensor tuned: brightness +2, saturation −2, AGC on, gain ceiling ×64, AEC off, vertical flip on.
5. WiFi connected (station mode, sleep disabled).
6. TCP server started on **port 11212**.

### Main loop (`loop()`)

Each iteration:

1. **Accept TCP client** — a new connection replaces any existing one (single-client model).
2. **IMU update** — `fusion.update(gyro, accel, vel_t)` runs every iteration using `accIntegral` (rcmags/imuFilter library). Roll/pitch/yaw and velocity (mm/s) are appended to a ring buffer (`imuFramesBuffer`, up to 128 frames × 6 floats).
3. **UDP discovery broadcast** — every 100 loop iterations the device broadcasts its IP address on **UDP port 11211** so clients can auto-discover it.
4. **TCP transmit** (when a client is connected):
   - Send one camera frame as `PACKET_TYPE_IMAGE` (header + JPEG bytes in 1 kB chunks).
   - Send all buffered IMU frames as `PACKET_TYPE_IMU` (header + raw float array), then reset the IMU buffer index.
   - Flush and close the connection — the device operates in a **connect-per-burst** mode rather than keeping a persistent stream.

### Wire protocol

All packets share a 9-byte little-endian header:

```
uint8_t  packet_type   // 0x01 = IMAGE, 0x02 = IMU
uint32_t frame_time    // millis() at send time
uint32_t total_size    // payload byte count
```

An IMU payload is `N × 24` bytes, each frame being six `float32` values: `roll, pitch, yaw, vx, vy, vz`.

IMU units: angles in radians (from `accIntegral`), velocity in mm/s (`GRAVITY = 9810 mm/s²`).

## Test client: `src/test_tcp_hud.py`

Python 3 script that exercises the full pipeline end-to-end.

```
python src/test_tcp_hud.py
```

**Discovery phase** — listens on UDP 11211 for the broadcast IP (called twice in the current code; the second call is a no-op duplicate).

**Receive loop** — reconnects to TCP 11212 on each burst:
- Reads the 9-byte header, then reads `total_size` bytes.
- `PACKET_TYPE_IMAGE`: decodes JPEG with OpenCV and displays it in a window (`q` to quit).
- `PACKET_TYPE_IMU`: unpacks and prints each `(roll, pitch, yaw, vx, vy, vz)` frame.

Dependencies: `opencv-python`, `numpy`.

## Network ports summary

| Port | Protocol | Direction | Purpose |
|------|----------|-----------|---------|
| 11211 | UDP broadcast | ESP32 → LAN | IP discovery |
| 11212 | TCP | host → ESP32 | Camera + IMU data retrieval |

## Relationship to ORB-SLAM3

This node sits under `Thirdparty/` and acts as the physical camera+IMU front-end for the ORB-SLAM3 monocular pipeline. The host-side localization service (see `localization_service/`) consumes the TCP stream, feeds JPEG frames into ORB-SLAM3, and uses the IMU velocity estimates to aid pose tracking and forced-pose initialization.

## Build & flash

```bash
# Install PlatformIO CLI or use the VS Code extension
pio run -t upload          # compile and flash
pio device monitor         # serial output at 115200 baud
```

WiFi credentials can be placed in `/config.txt` on the SD card (line 1 = SSID, line 2 = password) to avoid recompiling for different networks.
