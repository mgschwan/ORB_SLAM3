    # Project Summary - ORB_SLAM3 Modified

This project is a modified version of ORB-SLAM3. The primary goals of the modifications were to remove the mandatory dependency on Pangolin (for visualization) and to improve the "localization only" mode for robust use in pre-mapped environments.

## Project Directory Structure

```
ORB_SLAM3/
├── Agents.md                - This file (project documentation for AI agents)
├── CMakeLists.txt           - Modified build system (Pangolin now optional)
├── CMakeCache.txt           - Generated CMake cache
├── build.sh                 - Script to build the library and modified examples
├── build_ros.sh             - Script to build the ROS wrappers
├── mgschwan_changes.diff    - Git diff showing all modifications from upstream
├── Changelog.md             - Original ORB-SLAM3 changelog
├── Dependencies.md          - Library dependencies documentation
├── LICENSE                  - GPL v3 license
│
├── include/                 - C++ header files
│   ├── Atlas.h              - Multi-map container (+ SwitchToMap modification)
│   ├── Frame.h              - Single image frame with ORB features
│   ├── FrameDrawer.h        - Draws current frame (Pangolin-wrapped)
│   ├── KeyFrame.h           - Keyframe used in map
│   ├── KeyFrameDatabase.h   - BoW database for place recognition
│   ├── LocalMapping.h       - Local mapping thread
│   ├── LoopClosing.h        - Loop closure and map merging thread
│   ├── Map.h                - Single map instance (includes GL/glew.h fallback)
│   ├── MapDrawer.h          - 3D map visualization (Pangolin-wrapped)
│   ├── MapPoint.h           - 3D point in map
│   ├── ORBextractor.h       - ORB feature extractor
│   ├── ORBmatcher.h         - ORB feature matcher
│   ├── Optimizer.h          - G2O-based pose and map optimizers
│   ├── Settings.h           - Settings file loader
│   ├── System.h             - Main SLAM system interface (+ ForceRelocalization, GetAtlas)
│   ├── Tracking.h           - Tracking thread state machine (+ ForceRelocalization)
│   ├── Viewer.h             - Viewer class (all internals Pangolin-wrapped)
│   └── CameraModels/
│       ├── GeometricCamera.h
│       ├── KannalaBrandt8.h - Fisheye camera model
│       └── Pinhole.h        - Pinhole camera model
│
├── src/                     - C++ source files
│   ├── Atlas.cc             - Atlas implementation (+ SwitchToMap, debug prints)
│   ├── Frame.cc             - Frame implementation
│   ├── FrameDrawer.cc       - Frame drawing
│   ├── KeyFrame.cc          - Keyframe implementation
│   ├── KeyFrameDatabase.cc  - BoW database operations
│   ├── LocalMapping.cc      - Local mapping thread
│   ├── LoopClosing.cc       - Loop closure and map merging (mbActiveLC flag added)
│   ├── Map.cc               - Map implementation (debug prints + null-check fix)
│   ├── MapDrawer.cc         - Map drawing (Pangolin-wrapped)
│   ├── MapPoint.cc          - Map point implementation
│   ├── Optimizer.cc         - G2O optimizer
│   ├── ORBextractor.cc      - ORB feature extraction
│   ├── ORBmatcher.cc        - Feature matching
│   ├── Settings.cc          - Settings parsing
│   ├── System.cc            - System entry point (Pangolin removed, ForceRelocalization)
│   ├── Tracking.cc          - Tracking logic (key modification: no new map in loc mode)
│   ├── Viewer.cc            - Viewer thread (all Pangolin code wrapped/no-op'd)
│   ├── MLPnPsolver.cpp      - Perspective-n-Point solver for relocalization
│   └── CameraModels/
│       ├── KannalaBrandt8.cpp
│       └── Pinhole.cpp
│
├── localization_service/    - Main application (live camera SLAM / localization service)
│   ├── src/
│   │   └── localization_service_host.cc - [CUSTOM] Live camera stream with localization mode
│   ├── localization_service_host        - [BUILT EXECUTABLE]
│   ├── html/
│   │   ├── index.html       - Web frontend for SLAM control
│   │   ├── viewer.html      - Pose/map viewer
│   │   └── calibration.html - Camera calibration helper
│   ├── tools/
│   │   └── tello_camera_server.py - Tello drone camera relay server
│   └── example.yaml         - Example camera configuration
│
├── Examples/                - Active examples (only monocular built by modified CMake)
│   ├── Monocular/
│   │   ├── mono_tum_mgschwan.cc       - [CUSTOM] TUM dataset with localization mode
│   │   ├── mono_tum_mgschwan          - [BUILT EXECUTABLE]
│   │   ├── mgschwan.yaml              - Camera config (store map)
│   │   ├── mgschwan_localize.yaml     - Camera config (load map for localization)
│   │   ├── remote_cam_store.yaml      - Remote cam config (store map)
│   │   ├── remote_cam_load.yaml       - Remote cam config (load map)
│   │   ├── remote_droidcamx_store.yaml - Droidcam config (store map)
│   │   ├── remote_droidcamx_load.yaml  - Droidcam config (load map)
│   │   ├── sample_mon_mgschwan.sh     - Shell script to run mapping session
│   │   ├── sample_mon_mgschwan_localize.sh - Shell script to run localization
│   │   ├── *.osa                      - Saved Atlas files (binary map sessions)
│   │   └── [original TUM/KITTI/EuRoC sources kept but not built]
│   ├── Monocular-Inertial/  - IMU+Monocular examples (sources kept, not built)
│   ├── Stereo/              - Stereo examples (sources kept, not built)
│   ├── Stereo-Inertial/     - Stereo+IMU examples (sources kept, not built)
│   ├── RGB-D/               - RGB-D examples (sources kept, not built)
│   ├── RGB-D-Inertial/      - RGB-D+IMU examples (sources kept, not built)
│   └── Calibration/         - RealSense calibration recorders (sources kept, not built)
│
├── Examples_old/            - Original pre-restructuring examples (not built)
│
├── Thirdparty/              - Bundled third-party libraries
│   ├── DBoW2/               - Bag-of-Words place recognition library
│   ├── g2o/                 - Graph-based nonlinear optimization library
│   ├── Sophus/              - Lie group math library (SE3, SO3, Sim3)
│   └── orblsammer_espnode/  - ESP32-S3 firmware: camera+IMU WiFi sensor node
│       ├── src/xostudio_hud.ino   - PlatformIO firmware (camera+IMU TCP streamer)
│       ├── src/test_tcp_hud.py    - Python test client (UDP discovery + TCP receive)
│       └── platformio.ini         - PlatformIO build config (target: freenove_esp32_s3_wroom)
│
├── Vocabulary/
│   └── ORBvoc.txt.tar.gz   - ORB vocabulary for BoW (must be extracted before use)
│
├── evaluation/              - Python scripts for trajectory evaluation
│   ├── associate.py
│   ├── evaluate_ate_scale.py
│   └── Ground_truth/        - Ground truth trajectories for EuRoC datasets
│
└── lib/
    └── libORB_SLAM3.so      - Compiled shared library
```

## Major Changes

### 1. Pangolin Dependency Removal
- **Visualization Optional:** Pangolin is no longer required to build the core library.
- **Conditional Compilation:** Pangolin-dependent code in `Viewer`, `MapDrawer`, and `FrameDrawer` has been wrapped in `#if MGSCHWAN_DISABLED` blocks.
- **Build System:** `CMakeLists.txt` was updated to make Pangolin an optional dependency and to link against `GL` directly where needed.

### 2. Localization Mode Improvements
- **Map Preservation:** In the original ORB-SLAM3, if localization failed for a certain period, the system would often create a new map. This behavior has been disabled when `mbOnlyTracking` (Localization Mode) is active.
- **Persistent Relocalization:** When in localization mode, if the system loses track, it will now continuously attempt to relocalize against the existing map rather than starting a new one.
- **Manual Control:** Added `System::ForceRelocalization()` which sets the tracking state to `LOST`, triggering the relocalization logic.
- **Map Management:** Added `System::SwitchToMap(int idx)` to allow programmatically switching between different maps in the Atlas. This safely drains the `LocalMapping` queue, updates the Atlas, and instructs the Tracking thread to reset its state and relocalize without memory corruption.

### 3. Relocalization and Mapping Stability Fixes
- **Persistent Mapping Relocalization:** The automatic map creation feature when the system loses tracking in Mapping Mode has been heavily suppressed. It now continuously runs `Relocalization()` and tries to find its place in the current active map, mimicking Localization Mode's tenacity, instead of eagerly creating fragmented new maps.
- **KeyFrameDatabase Scope Accuracy:** Fixed a severe bug in `KeyFrameDatabase::DetectRelocalizationCandidates` that allowed Bag-of-Words queries to cross-pollinate with scores from inactive maps. It now properly bounds word scoring to the active map, allowing successful relocalization after switching maps.
- **Robust LoopClosing:** Ensured `LoopClosing::Run()` still correctly harvests and inserts newly generated `KeyFrames` into the `KeyFrameDatabase` even if the `loopClosing: 0` flag is set in the config. This prevents the database from remaining entirely empty and bricking relocalization functionality.
- **KeyFrame Culling Safety:** Hardened `KeyFrame::ChangeParent` to tolerate empty spanning trees/deleted parents, mitigating segmentation faults when the local map culler prunes bad KeyFrames post-map-swap.

## Deeper Inspection: Tracking Logic

The tracking logic resides primarily in `src/Tracking.cc` and is governed by a state machine (`mState`).

### Tracking States (`eTrackingState`)
- `NO_IMAGES_YET`: Initial state before any processing.
- `NOT_INITIALIZED`: Sensor is active but the initial map has not been created.
- `OK`: System is tracking successfully against the map.
- `RECENTLY_LOST`: Tracking just failed. The system attempts quick recovery (e.g., using IMU or a short-window search).
- `LOST`: Tracking failed for an extended period (usually > 3s). In standard SLAM, this triggers a new map creation.

### Main Tracking Flow (`Tracking::Track`)
Each frame goes through the following stages:
1. **Pose Prediction**: 
   - **Motion Model**: If the camera is moving with consistent velocity, the pose is predicted using `mVelocity`.
   - **IMU Prediction**: If an IMU is present and initialized, it predicts the pose.
   - **Reference KeyFrame**: If the motion model fails, it tries to match features against the last reference KeyFrame.
2. **Local Map Tracking (`TrackLocalMap`)**: 
   - This is the most crucial step for accuracy. It retrieves "neighboring" KeyFrames from the covisibility graph and projects their MapPoints into the current frame to find more matches and refine the pose via `PoseOptimization`.
3. **Relocalization**: 
   - Triggered when `mState` is `LOST` or `RECENTLY_LOST`. It uses Bag-of-Words (BoW) to find candidate KeyFrames in the database and computes a pose using the `MLPnP` solver.

### Localization-Only Mode (`mbOnlyTracking`)
When activated via `SLAM.ActivateLocalizationMode()`:
- **`NeedNewKeyFrame()`**: Always returns `false`, preventing the Local Mapping thread from adding new KeyFrames.
- **Visual Odometry (`mbVO`)**: If the system loses track of the global map but still sees enough temporal points, it enters a VO mode. It continues tracking relatively while simultaneously trying to relocalize back to the global map.
- **Modified `LOST` Behavior**: In the modified codebase, the logic in `Tracking::Track()` that usually calls `CreateMapInAtlas()` is bypassed if `mbOnlyTracking` is true. This forces the system to stay in the `LOST` state and keep calling `Relocalization()` on every new frame until it succeeds.

### Optimization and PnP
- **`PoseOptimization`**: A G2O-based bundle adjustment that refines only the camera pose while keeping MapPoints fixed.
- **`MLPnPsolver`**: Used during relocalization; it requires at least 50 inliers (by default) to validate a relocalization attempt.

### Key Data Structures
- **`Frame`**: Represents a single image with extracted ORB features. It's a temporary object unless it's promoted to a KeyFrame.
- **`KeyFrame`**: A frame that has been added to the map. It stores:
    - BoW vector for relocalization and loop closing.
    - Camera pose (Tcw).
    - Covisibility information (which other KFs see the same points).
- **`MapPoint`**: A 3D point in the world. It stores:
    - World position.
    - View direction and depth range.
    - A "representative" descriptor (chosen from all observations).
- **`Atlas`**: The multi-map manager. It contains a set of `Map` objects. Only one map is "current" at a time.

### 4. New Executables
- **`Examples/Monocular/mono_tum_mgschwan.cc`**:
    - Supports an optional `localize_only` command-line argument.
    - Specifically designed to work with TUM dataset format but with the improved localization logic.
- **`localization_service/src/localization_service_host.cc`** (built to `localization_service/localization_service_host`):
    - Uses `cv::VideoCapture` to process streams from a URL (e.g., IP camera, MJPEG stream). Now also handles local V4L2 device formats directly (e.g., `/dev/video0`).
    - Passing `none` as the camera source skips `VideoCapture` entirely; frames are then supplied exclusively via `POST /api/frame`.
    - Acts as an asynchronous Web Interface for manual control over SLAM operations.
    - Features endpoints for REST API Status (`/api/status`) serving JSON variables, action endpoints (`/pause`, `/resume`, `/newmap`, `/switchmap?id=X`), and safely serves static frontend assets (HTML, CSS, JS) directly out of `localization_service/html/`.
    - Frame ingest endpoint `POST /api/frame`: accepts a raw JPEG body and optional `ts`/IMU query parameters. Returns a JSON response with `queued`, `tracking_state`, and `pose` fields. An empty-body POST skips frame submission and returns the current pose snapshot — used for polling after a frame has been queued.

- **`localization_service/tools/send_camera_frames.py`**:
    - Forwards frames from a local camera (V4L2, file, or URL) to the frame ingest API.
    - Displays the pose returned in each POST response inline in the terminal.
    - Includes `query_pose()` helper and a commented single-shot localization pattern at the bottom of the file.

### 5. Configuration & Settings
- **Loop Closing Toggle:** Added support for a `loopClosing` flag in the YAML settings file to enable/disable the Loop Closing thread.
- **Image Scaling:** Improved handling of image scaling in the examples to match the SLAM system's expectations.

## Technical Details

### Localization Mode Activation
To activate the improved localization mode in your code:
```cpp
SLAM.ActivateLocalizationMode();
SLAM.SwitchToMap(map_id);            // Optional: Safely switch to a specific saved map
SLAM.ForceRelocalization();          // Force the system to start searching in the map
```

### Tracking Logic Change (`Tracking.cc`)
The core change preventing new map creation is in `Tracking::Track()`:
```cpp
// Original logic would create a new map if lost
else if (mState == LOST)
{
    // MODIFIED: Do not automatically create a new map.
    // Instead, continuously try to relocalize in the current map.
    Verbose::PrintMess("State is LOST. Continuously trying to relocalize...", Verbose::VERBOSITY_NORMAL);
    bOK = Relocalization();
    if (!bOK) {
        return;
    }
}
```

### Frame Ingest API (`POST /api/frame`)

The frame ingest path allows an external process to push frames into the tracking loop instead of reading from a camera device. The camera source must be `"none"` when launching the service.

**Key types** (`localization_service/include/localization_service/ingest_queue.h`):
- `IngestFrame` — holds a `cv::Mat image`, `double timestamp` (ms), and optional IMU fields (`hasImu`, `ax/ay/az`, `gx/gy/gz`).
- `IngestQueue` — thread-safe bounded queue (max depth 2). `push()` returns `false` if full (caller maps this to HTTP 503). `pop()` blocks with a timeout and is called from the main tracking loop when no `VideoCapture` is open.

**Request** — `POST /api/frame`:
- Body: raw JPEG bytes with `Content-Type: image/jpeg`
- Empty / `Content-Length: 0` body: pose-only query, no frame submitted
- Query params: `ts` (ms), `ax`, `ay`, `az`, `gx`, `gy`, `gz`

**Response** — always JSON:
```json
{ "queued": true, "tracking_state": "OK",
  "pose": { "valid": true, "x": 1.2, "y": 0.3, "z": 0.9,
            "qx": 0, "qy": 0, "qz": 0, "qw": 1 } }
```
- `queued: true` — frame was decoded and accepted into the ingest queue
- `queued: false` — empty-body request; pose snapshot only
- HTTP 503 — queue full; client should drop the frame and retry

**Single-shot localization pattern** (post one frame, poll until confirmed):
```python
# 1. Submit frame — response contains the pose from the *previous* frame.
r = session.post(url, data=jpeg_bytes, headers={"Content-Type":"image/jpeg"},
                 params={"ts": f"{time.monotonic()*1000:.3f}"})

# 2. Empty-body poll — wait for this frame to be processed.
for _ in range(20):
    r = session.post(url, data=b"")
    d = r.json()
    if d["tracking_state"] == "OK" and d["pose"]["valid"]:
        break
    time.sleep(0.05)
```

**Implementation files:**
- `localization_service/include/localization_service/ingest_queue.h` — types
- `localization_service/src/ingest_queue.cc` — `push` / `pop` implementation
- `localization_service/src/web_server.cc` — `routeIngest`, `handleFramePost`, `makePoseJson`
- `localization_service/src/localization_service_host.cc` — `IngestQueue` creation; main loop branches on `useIngest`

### Build Requirements
- OpenCV
- Eigen3
- Sophus (included in Thirdparty)
- DBoW2 & g2o (included in Thirdparty)
- (Optional) Pangolin

## Known Build and Runtime Issues

### SIGSEGV in g2o SE3Quat push_back during Bundle Adjustment (GCC 13)

**Symptom:** The process crashes with a segmentation fault inside
`std::vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>>::push_back`
during the initial map creation (`Tracking::CreateInitialMapMonocular` →
`Optimizer::GlobalBundleAdjustment`). A GDB backtrace will show the `this`
pointer of the `SE3Quat` constructor pointing into `main_arena` (glibc's
internal malloc state), e.g.:

```
#0  g2o::SE3Quat::SE3Quat (this=0x7ffff6803b50 <main_arena+144>) ...
#6  g2o::BaseVertex<6, g2o::SE3Quat>::push ...
#13 ORB_SLAM3::Tracking::CreateInitialMapMonocular ...
```

**Root cause:** GCC 13 emits a false-positive `-Warray-bounds` warning for SSE
intrinsics reading Eigen fixed-size arrays (triggered in `Sophus/sophus/sim2.hpp`
via `rxso2().params()`). Because Sophus's `CMakeLists.txt` compiles with
`-Werror`, this turns the warning into a compilation error. On systems where
Sophus was previously compiled with an older GCC that did not emit the warning,
the Sophus build appears to succeed but the test binary is left in a broken state
that can corrupt the heap at runtime when the g2o aligned allocator is exercised.
Recompiling on GCC 13 without the fix causes a hard build error that reveals the
problem.

**Fix:** Two changes are required (both already applied in this repo):

1. `Thirdparty/Sophus/CMakeLists.txt` — add `-Wno-array-bounds` to the GCC
   flags so the false-positive is suppressed:
   ```cmake
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -std=c++11
       -Wno-deprecated-declarations -Wno-array-bounds -ftemplate-backtrace-limit=0")
   ```

2. `build.sh` — pass `-DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF` when configuring
   Sophus, since ORB-SLAM3 only needs the Sophus headers:
   ```bash
   cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
   ```

**When rebuilding on a new system:** always delete `Thirdparty/Sophus/build/`
before re-running `build.sh` so CMake picks up the updated flags:
```bash
rm -rf Thirdparty/Sophus/build
./build.sh
```

## Usage
The new executables are output to their respective source directories after compilation.
Example for `localization_service_host`:
```bash
./localization_service/localization_service_host path_to_vocabulary path_to_settings camera_url [localize_only] [map_id]
```

---

## ESP32-S3 Sensor Node (`Thirdparty/orblsammer_espnode/`)

An ESP32-S3 firmware + test client that acts as the physical camera and IMU front-end for the localization pipeline. Frames and IMU data arrive over WiFi TCP; the host localization service feeds them into ORB-SLAM3.

### Hardware

| Component | Detail |
|-----------|--------|
| MCU | Freenove ESP32-S3 WROOM (has PSRAM) |
| Camera | OV5640 — `CAMERA_MODEL_ESP32S3_EYE` pin mapping, QVGA/JPEG quality 12 |
| IMU | MPU-6050 on I2C (SDA=GPIO21, SCL=GPIO20, 400 kHz) — roll/pitch/yaw + velocity |
| Storage | SD/MMC (CLK=39, CMD=38, D0=40) — optional, holds WiFi credentials |

### Firmware: `src/xostudio_hud.ino`

Built with PlatformIO (`platformio.ini`, target `freenove_esp32_s3_wroom`).

**Setup sequence:**
1. I2C + MPU-6050 init; `imu.setBias()` after a 5-second still period.
2. SD card mounted; reads SSID/password from `/config.txt` (lines 1 & 2) if present, otherwise uses compile-time defaults.
3. Camera init: PSRAM frame buffer, `CAMERA_GRAB_LATEST`, brightness +2, saturation −2, AGC on, gain ceiling ×64, AEC off, vertical flip on.
4. WiFi connected (station mode, sleep disabled).
5. TCP server started on **port 11212**.

**Main loop** (connect-per-burst model, single client):
1. Accept TCP client — new connection replaces any existing one.
2. IMU update — `fusion.update()` every iteration; readings accumulate in a ring buffer (up to 128 frames × 6 floats).
3. UDP discovery broadcast — every 100 iterations, broadcasts the device IP on **UDP port 11211**.
4. TCP transmit — one camera frame as `PACKET_TYPE_IMAGE`, then all buffered IMU frames as `PACKET_TYPE_IMU`; then close.

### Wire Protocol

All packets share a **9-byte little-endian header**:

```
uint8_t  packet_type   // 0x01 = IMAGE, 0x02 = IMU
uint32_t frame_time    // millis() at send time
uint32_t total_size    // payload byte count
```

An IMU payload is `N × 24` bytes; each frame is six `float32` values:

```
roll, pitch, yaw  (radians, from accIntegral)
vx, vy, vz        (mm/s, GRAVITY = 9810 mm/s²)
```

### Network Ports

| Port | Protocol | Direction | Purpose |
|------|----------|-----------|---------|
| 11211 | UDP broadcast | ESP32 → LAN | IP auto-discovery |
| 11212 | TCP | host → ESP32 | Camera frame + IMU retrieval |

### Test Client: `src/test_tcp_hud.py`

Python 3 script for end-to-end testing (requires `opencv-python`, `numpy`):

```bash
python src/test_tcp_hud.py
```

- **Discovery phase:** listens on UDP 11211 for the broadcast IP.
- **Receive loop:** reconnects to TCP 11212 each burst; decodes `PACKET_TYPE_IMAGE` with OpenCV (displays in a window, `q` to quit) and prints `PACKET_TYPE_IMU` tuples.

### Build & Flash

```bash
# Install PlatformIO CLI or use the VS Code extension
pio run -t upload      # compile and flash
pio device monitor     # serial output at 115200 baud
```

WiFi credentials go in `/config.txt` on the SD card (line 1 = SSID, line 2 = password) to avoid recompiling for different networks.
