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
│   ├── ORBextractor.h       - ORB feature extractor (implements FeatureExtractor)
│   ├── FeatureExtractor.h   - Abstract feature extractor interface + FeatureType enum + factory
│   ├── AkazeExtractor.h     - AKAZE feature extractor (implements FeatureExtractor)
│   ├── ORBmatcher.h         - Feature matcher (DescriptorDistance generalized to any binary length)
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
│   ├── FeatureExtractor.cc  - Feature extractor factory + FeatureType helpers
│   ├── AkazeExtractor.cc    - AKAZE feature extraction
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
│   │   ├── localization_service_host.cc - [CUSTOM] Main entry point: arg parsing, tracking loop
│   │   ├── espnode_source.cc            - [CUSTOM] ESP32 TCP session + ImuBuffer
│   │   ├── slam_state.cc                - Shared flags and pose state
│   │   ├── ingest_queue.cc              - IngestQueue push/pop
│   │   ├── calibration_manager.cc       - Chessboard calibration logic
│   │   ├── preprocessor.cc              - [CUSTOM] Frame preprocessing pipeline (CLAHE, extensible)
│   │   └── web_server.cc                - HTTP server and all route handlers
│   ├── include/localization_service/
│   │   ├── args.h               - [CUSTOM] ServiceArgs, parseArgs() — all CLI flags
│   │   ├── config.h             - Port and tuning constants
│   │   ├── slam_state.h         - LifecycleFlags, PoseState
│   │   ├── ingest_queue.h       - IngestFrame, IngestQueue
│   │   ├── espnode_source.h     - [CUSTOM] ImuSample, ImuBuffer, EspnodeSource
│   │   ├── calibration_manager.h
│   │   ├── preprocessor.h         - [CUSTOM] PreprocessStep, FramePreprocessor
│   │   └── web_server.h
│   ├── localization_service_host        - [BUILT EXECUTABLE]
│   ├── html/
│   │   ├── index.html       - Web frontend for SLAM control
│   │   ├── viewer.html      - Pose/map viewer
│   │   └── calibration.html - Camera calibration helper
│   ├── tools/
│   │   ├── tello_camera_server.py - Tello drone camera relay server
│   │   ├── send_camera_frames.py  - Forward a local camera to the frame ingest API
│   │   ├── record_frames.py       - Record frames to disk for offline processing
│   │   ├── replay_frames.py       - Replay recorded frames into the ingest API
│   │   ├── osa_convert.cc         - [CUSTOM] C++ CLI: dump/pack OSA ↔ JSON
│   │   ├── osa_convert            - [BUILT EXECUTABLE]
│   │   ├── osa_file.py            - [CUSTOM] Python module: read/write OSA atlas files
│   │   ├── akaze_vocab_trainer.cc - [CUSTOM] C++ CLI: train an AKAZE DBoW2 vocabulary
│   │   └── akaze_vocab_trainer    - [BUILT EXECUTABLE]
│   ├── example.yaml         - Example camera configuration (ORB)
│   └── example_akaze.yaml   - Example camera configuration (AKAZE feature type)
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

## OSA File Format and Offline Atlas Tools

### OSA file format

An `.osa` file is a **Boost binary archive** produced by `boost::archive::binary_oarchive`. The layout is:

1. `std::string vocabName` — vocabulary filename (e.g. `"ORBvoc.txt"`)
2. `std::string vocabChecksum` — MD5/SHA of the vocabulary
3. `Atlas* mpAtlas` — pointer-serialized via `boost::serialization`; the Atlas recursively serializes its Maps, KeyFrames, and MapPoints using `PreSave`/`PostLoad` pointer-to-ID transformations. Atlas serialization is at `BOOST_CLASS_VERSION 1`, which adds an `mnFeatureType` field (0=ORB, 1=AKAZE, 2=SIFT); `version 0` archives (pre-swappable-features) load as ORB. Descriptors are stored width-agnostically, so ORB (32-byte) and AKAZE (32- or 61-byte) round-trip through the same code.

All major classes use `template<class Archive> void serialize(Archive&, unsigned int)`. Key serialization utilities live in `include/SerializationUtils.h` (`serializeSophusSE3`, `serializeMatrix`, `serializeVectorKeyPoints`).

### `osa_convert` (C++ tool — `localization_service/tools/osa_convert.cc`)

A standalone command-line tool that converts an OSA file to/from an intermediate JSON representation.

```bash
./localization_service/tools/osa_convert dump  <input.osa>  <output.json>
./localization_service/tools/osa_convert pack  <input.json> <output.osa>
```

**`dump`** walks the Atlas hierarchy and writes all serialized fields as JSON. Binary blobs (cv::Mat descriptors, Sophus SE3, KeyPoints) are base64-encoded. NaN/inf floats (common in uninitialized IMU fields for monocular maps) are sanitized to 0.0.

**`pack`** reconstructs a fully loadable Atlas from the JSON. Cameras are re-created via their constructors (which auto-assign IDs via `GeometricCamera::nNextId`). KeyFrame/MapPoint fields that are `const` or `protected` are set via `const_cast` and wrapper subclass setters.

Build target (Ninja):
```bash
cd build && ninja osa_convert
```

**Implementation notes for agents:**
- Wrapper subclasses (`MapWrapper`, `AtlasWrapper`, `KeyFrameAccess`, `MapPointAccess`) expose `protected` members and provide `const_cast` setters
- `IMU::Preintegrated` is not copyable (contains `std::mutex`); it must be filled in-place via `fill_imu_preint(json, IMU::Preintegrated&)`
- Camera IDs: reset `GeometricCamera::nNextId = 0` before creating cameras; constructors auto-assign `mnId = nNextId++`; restore afterwards
- The project uses C++14 — no structured bindings (`auto& [k, v]`)

### `osa_file.py` (Python module — `localization_service/tools/osa_file.py`)

A pure-Python module for reading and writing OSA files. It shells out to `osa_convert` for the actual serialization and deserializes the JSON into typed dataclasses backed by NumPy arrays.

```python
from localization_service.tools.osa_file import OsaAtlas

# Load an existing atlas
atlas = OsaAtlas.load("Session.osa")

# Inspect
m = atlas.maps[0]
pts   = m.world_points()        # (N, 3) float32 — all MapPoint positions
cams  = m.camera_centers()      # (K, 3) float32 — all KeyFrame camera centers
kf    = m.keyframe_by_id(42)    # OsaKeyFrame | None
mp    = m.mappoint_by_id(123)   # OsaMapPoint | None

# Round-trip save
atlas.save("output.osa")

# Create a blank atlas for building from scratch
atlas = OsaAtlas.new("ORBvoc.txt", "checksum_string")

# Dump intermediate JSON for debugging
atlas.to_json_file("debug.json")
```

**Key dataclasses:**
- `OsaAtlas`: `vocab_name`, `vocab_checksum`, static ID fields, `cameras` (list of `OsaCamera`), `maps` (list of `OsaMap`)
- `OsaMap`: `id`, `keyframes` (list of `OsaKeyFrame`), `mappoints` (list of `OsaMapPoint`)
- `OsaKeyFrame`: `id`, `timestamp`, `tcw` (4×4 float32 camera-to-world), `fx/fy/cx/cy`, `descriptors` (N×32 uint8), `keypoints`, `bow_vec`, `feat_vec`, `mappoint_ids`, `grid`, `connected_kf_weights`, spanning tree fields, IMU fields
- `OsaMapPoint`: `id`, `first_kf_id`, `world_pos` (float32[3]), `normal` (float32[3]), `descriptor` (uint8[32]), `obs1`/`obs2` dicts, `min_dist`, `max_dist`

**Helper functions** (in `osa_file.py`):
- `build_feature_grid(keypoints, width, height, grid_cols, grid_rows)` — assigns keypoints to spatial grid cells
- `make_scale_pyramid(scale_levels, scale_factor)` — returns `(factors, sigma2, inv_sigma2, log_factor)`
- `_default_imu_preint()` / `_default_imu_calib()` — zero-filled IMU defaults for monocular (non-inertial) atlases

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
- **Mutually-exclusive localization-mode toggles:** `System::ActivateLocalizationMode()` / `DeactivateLocalizationMode()` only set their own deferred flag (`mbActivate/DeactivateLocalizationMode`), consumed later inside `TrackMonocular`. A stale, not-yet-consumed flag could survive (e.g. the atlas **download** handler temporarily activates then deactivates localization for a safe export). If the user then switched to localization mode before any frame was processed, **both** flags were set, and `TrackMonocular` ran activate **then** deactivate in the same frame — silently leaving the system in **mapping** mode (`mbOnlyTracking==false`). This in turn defeated the localization-mode guard on map creation (a stale-timestamp frame then created a new map). Fix: each setter now clears the opposite pending flag, so the **latest** requested mode always wins and the two flags can never both be set.
- **No map creation / arbitrary timestamps in localization mode:** The timestamp-jump handler in `Tracking::Track()` ([Tracking.cc](src/Tracking.cc), ~line 1913) called `CreateMapInAtlas()` whenever a frame arrived with a timestamp older than the previous frame (and `ResetActiveMap()`/`CreateMapInAtlas()` on large forward jumps for inertial maps). In localization mode this is wrong: a stale/out-of-order timestamp would silently switch the system into map creation. The whole timestamp-jump block is now guarded with `!mbOnlyTracking`, so in localization mode the system never resets/creates a map and tolerates **arbitrary, non-monotonic timestamps** (different devices localizing against the same map). The motion model is pose-based (not divided by `dt`), so out-of-order timestamps don't destabilize it, and tracking falls back to relocalization when temporal continuity breaks. (The other reachable `CreateMapInAtlas()` in `Track()` is already gated by `mState==LOST && !mbOnlyTracking`.)
- **Null reference-KeyFrame after map switch (segfault fix):** `InformMapSwitch()` deliberately clears `mpReferenceKF` and sets `mState = RECENTLY_LOST` so the tracker relocalizes into the freshly switched/loaded map. But the **localization-mode** branch of `Tracking::Track()` only relocalizes when `mState == LOST`; for `RECENTLY_LOST` with no velocity it called `TrackReferenceKeyFrame()`, which dereferenced the now-null `mpReferenceKF` (`SearchByBoW(pKF=0x0)` → `KeyFrame::GetMapPointMatches(this=0x0)`). This crashed reliably when an atlas was loaded via the web UI **while already in localization mode** (load happens after `ActivateLocalizationMode`, so the first post-load frame hits the null ref). Fix: in the loc-mode branch, relocalize when `mpReferenceKF == nullptr`, and harden `TrackReferenceKeyFrame()` to return `false` on a null reference KF. Unrelated to the target-resolution scaling work (the `- Target scale:` log line merely preceded the crash).

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
    - **Camera sources**: V4L2 device, MJPEG/RTSP URL, `none` (HTTP push), `espnode` / `espnode:<ip>` (ESP32 sensor node).
    - **CLI**: all optional arguments are named flags parsed by `args.h::parseArgs()`. No positional optional arguments.
    - Passing `none` as the camera source uses the `IngestQueue` path; frames are supplied via `POST /api/frame`.
    - Passing `espnode` auto-discovers the ESP32 via UDP and opens a persistent TCP session (`EspnodeSource`). Frames arrive via trigger-response; IMU data streams continuously into `ImuBuffer` and is drained once per tracking frame.
    - Acts as an asynchronous Web Interface for manual control over SLAM operations.
    - Features endpoints for REST API Status (`/api/status`) serving JSON variables, action endpoints (`/pause`, `/resume`, `/newmap`, `/switchmap?id=X`, `/allow_new_maps?enable=`, `/use_motion_model?enable=`), and safely serves static frontend assets (HTML, CSS, JS) directly out of `localization_service/html/`.
    - Frame ingest endpoint `POST /api/frame`: accepts a raw JPEG body and optional `ts`/IMU query parameters. Returns a JSON response with `queued`, `tracking_state`, and `pose` fields. An empty-body POST skips frame submission and returns the current pose snapshot — used for polling after a frame has been queued.

- **`localization_service/tools/send_camera_frames.py`**:
    - Forwards frames from a local camera (V4L2, file, or URL) to the frame ingest API.
    - Displays the pose returned in each POST response inline in the terminal.
    - Includes `query_pose()` helper and a commented single-shot localization pattern at the bottom of the file.

### 5. Configuration & Settings
- **Loop Closing Toggle:** Added support for a `loopClosing` flag in the YAML settings file to enable/disable the Loop Closing thread.
- **Swappable feature extractor:** New `Feature.type` YAML key (`ORB` | `AKAZE`) selects the feature detector/descriptor for the whole atlas. See the dedicated subsection below.
- **Motion-model toggle:** New `Tracking.useMotionModel` YAML key (default `1`) plus a runtime switch to disable the constant-velocity motion model. See the dedicated subsection below.
- **Image Scaling:** Improved handling of image scaling in the examples to match the SLAM system's expectations.
- **Target Processing Resolution (dynamic scaling):** New `Camera.targetWidth` / `Camera.targetHeight` YAML keys. When set, incoming frames of any resolution are uniformly scaled to the target *inside* `Tracking::GrabImageMonocular`, so the service can accept different cameras without per-camera tuning. Works with both config formats. See below.

#### Target-resolution downscaling (`Camera.targetWidth` / `Camera.targetHeight`)

Goal: process at one fixed internal resolution regardless of the incoming camera resolution, while keeping every external interface (YAML intrinsics, REST calibration, chessboard capture) expressed in the camera's **native** resolution. Clients never need to know the internal processing size.

**Config-format independence (important):** the keys are parsed in the **Tracking constructor** (`ParseTargetResolution`), *before* the camera loader runs, so the feature works for **both** config paths — the legacy `ParseCamParamFile` path **and** the `File.version: "1.0"` `newParameterLoader(Settings*)` path that all the service YAMLs actually use. (An earlier draft only wired it into `ParseCamParamFile`, which those YAMLs never call — `Tracking::Tracking` calls `newParameterLoader` when `settings != nullptr`.) `TargetModeActive()` = `mTargetWidth>0 || mTargetHeight>0`.

**Mechanism (all in `src/Tracking.cc`):**
- `ParseTargetResolution(fSettings)` (constructor): reads `Camera.width/height` (calibration reference, used only for the mismatch warning) and `Camera.targetWidth/targetHeight`. Either or both target dims may be set.
- Native source intrinsics are captured **lazily** on the first frame by `CaptureSourceIntrinsics()`, reading `fx/fy/cx/cy` (+ distortion / type) straight from whichever `mpCamera` the loader built. So neither loader needs target-specific code; they only **skip** the legacy `Camera.imageScale` baking when `TargetModeActive()` (keeping intrinsics native).
- `GrabImageMonocular` (target mode only): computes a single **uniform** scale — `min(targetW/fw, targetH/fh)` with both dims, or the single-axis ratio with one — `cv::resize`s the frame to `round(fw·s) × round(fh·s)` (**no padding**), and lazily rebuilds the scaled camera model via `RebuildScaledCamera(s)` whenever the frame size or calibration changes. `mImageScale` holds the live `s`. Frames smaller than the target are upscaled (`s > 1`) so the processing resolution always equals the target, keeping query feature scale consistent with the map descriptors.
- `RebuildScaledCamera(float s)`: rebuilds `mpCamera` / `mK` / `mK_` from the source intrinsics scaled by `s`. **Distortion coefficients are left unchanged** — they are scale-invariant in normalized coordinates, and ORB-SLAM3 monocular undistorts feature *coordinates* (`Frame::UndistortKeyPoints`) after extraction, not the image, so scale-then-undistort is geometrically exact.
- `ChangeCalibration(K, D, type)` (REST path): in target mode it stores the supplied **native** K as new source intrinsics, sets `mbSrcCaptured` (so the lazy capture won't clobber it), `mbManualCalib` (suppresses the mismatch warning) and `mbScaleDirty`, and returns — the scaled rebuild then happens on the tracking thread on the next frame. Legacy (non-target) behaviour is unchanged.
- Resolution-mismatch warning: emitted once when no REST calibration is active and the frame size differs from `Camera.width/height`; the frame resolution is then assumed correct for the intrinsics.

**Relation to `Camera.newWidth`/`newHeight`:** the stock `Settings` class has a separate, pre-existing resize (`Settings::readImageInfo` → `System::TrackMonocular`) that scales **non-uniformly** to a fixed `newImSize_` and assumes a fixed input resolution. The target keys here are the uniform, runtime-adaptive replacement; do not set both. `needToResize()` is only triggered by `newWidth/newHeight`, so target-only YAMLs don't hit the `Settings` resize.

**Accessors:** `Tracking::GetTargetSize(w,h)` → `System::GetTargetSize(w,h)` (0,0 when disabled; a single axis may be 0).

**Host (`localization_service_host.cc`):** in target mode the host does **not** resize; it passes native frames to both `calib.processFrame` (so calibration sees the original frame) and `slam.TrackMonocular`. The legacy `imageScale` resize is guarded to the non-target path. `calibration_manager.cc` and `web_server.cc` are unchanged.

When the target keys are unset, the legacy fixed `Camera.imageScale` path (intrinsics scaled at parse time, image resized by the caller) is used exactly as before.

#### Frame preprocessing pipeline (`Preproc.*`)

An extensible, ordered image preprocessing pipeline in the host (`localization_service/src/preprocessor.{h,cc}`), applied to each frame **after** the calibration capture (which keeps using the raw frame) and **before** `slam.TrackMonocular`. Both mapping and localization run through it, so ORB descriptors stay consistent.

- `FramePreprocessor::fromSettingsFile(settingsPath)` opens the YAML and builds the enabled steps from `Preproc.*` keys; `process(cv::Mat&)` runs them in order; `describe()` logs the pipeline at startup. The host builds it once after `System` construction and calls `preproc.process(frame)` before tracking.
- `PreprocessStep` is the per-step interface (`apply(cv::Mat&)` in place, `describe()`).
- **CLAHE** (`ClaheStep`) is the first step: `Preproc.clahe` (0/1), `Preproc.claheClipLimit` (default 2.0), `Preproc.claheTileSize` (default 8). Colour frames get CLAHE on the LAB **L channel** (colour preserved); grayscale is equalized directly. Default is **off** unless `Preproc.clahe: 1`, so existing YAMLs are unaffected.
- **Adding a step later:** implement a `PreprocessStep` subclass in `preprocessor.cc`, then read its `Preproc.*` keys and append it in `fromSettingsFile()` at the desired pipeline position. Add the new `.cc` only if split out; `preprocessor.cc` is already in the `CMakeLists.txt` host target.
- Caveat: preprocessing alters pixel intensities and therefore descriptors — a map built with a given pipeline must be localized with the same pipeline.

#### Motion-model toggle (`Tracking.useMotionModel`)

The constant-velocity motion model predicts the next pose as `mVelocity * mLastFrame.pose` (where `mVelocity` is the relative transform between the last two frames) and then only searches for matches near that prediction (`TrackWithMotionModel` → `SearchByProjection`). With **far-apart frames** the constant-velocity assumption breaks and the prediction overshoots; worse, right after a `Relocalization()` the velocity is recomputed from the *previous* (stale/wrong) frame pose, so **the frame following a correct relocalization is often placed completely wrong**.

`Tracking::mbUseMotionModel` (default **true**) gates this. When **false**:
- Every frame is tracked from the **last known pose** via `TrackReferenceKeyFrame()` (which seeds `mCurrentFrame.SetPose(mLastFrame.GetPose())` and matches against the reference keyframe by BoW), then refined by `TrackLocalMap()`.
- If reference-keyframe tracking fails, a global `Relocalization()` is attempted **in the same frame** (the "enhanced" fallback) rather than waiting for the next frame's RECENTLY_LOST/VO path.
- Applies to **both** branches of `Tracking::Track()` — mapping (`!mbOnlyTracking`, ~line 2103) and localization (`mbOnlyTracking`, ~line 2215) — and the VO-recovery motion-model fallback is likewise skipped. The external/forced-pose path (`mbHasForcedPose`) is unaffected (it bypasses prediction entirely).

**Plumbing** (mirrors the `allow_new_maps` toggle end-to-end):
- `Tracking`: `mbUseMotionModel` + `SetUseMotionModel`/`GetUseMotionModel`; read from `Tracking.useMotionModel` in the constructor (same block as `Feature.type`, so it applies to both config formats).
- `System`: `SetUseMotionModel` / `GetUseMotionModel` passthrough.
- Host (`localization_service_host.cc`): `std::atomic<bool> useMotionModel{slam.GetUseMotionModel()}` (seeded from the YAML), passed to `WebServer`; `motion_on` / `motion_off` stdin commands.
- Web (`web_server.cc`): `GET /use_motion_model?enable=true|false`; `useMotionModel` field in `/api/status`. UI: a **Motion Model** toggle button in `html/index.html`.

#### Swappable feature extractor (`Feature.type`: `ORB` | `AKAZE`)

The feature detector/descriptor is selectable behind a `FeatureExtractor` interface. An atlas uses **one uniform feature type**; it is chosen in the YAML and recorded in the serialized atlas, so a map is always localized with the type it was built with.

**Architecture:**
- `FeatureExtractor` (`include/FeatureExtractor.h`) is the abstract base: `operator()` (extract keypoints + descriptors), the scale-pyramid getters Frame consumes, `GetType()`, and the shared `mvImagePyramid` (used only by stereo matching). `ORBextractor` and `AkazeExtractor` implement it. `CreateFeatureExtractor(type, …)` (in `src/FeatureExtractor.cc`) is the factory; `FeatureTypeFromString` / `FeatureTypeName` parse/print the type.
- `Frame` and `Tracking` hold `FeatureExtractor*` (not `ORBextractor*`); extractors are built via the factory in **both** config paths (`newParameterLoader` and `ParseORBParamFile`). `Tracking` reads `Feature.type` and the `AKAZE.*` keys near the top of its constructor (so they apply to both config formats) and exposes `GetFeatureType()`.
- `ORBmatcher::DescriptorDistance` is generalized to **arbitrary binary length** (a `cols/4` int32 fast path + trailing-byte popcount). ORB (32 bytes) is bit-identical to before; AKAZE works at any byte length. It is a **Hamming** metric — only valid for binary descriptors (ORB, AKAZE-MLDB).
- **Serialization:** `Atlas` stores `mnFeatureType` (0=ORB, 1=AKAZE, 2=SIFT), serialized under `BOOST_CLASS_VERSION(Atlas, 1)`. Pre-versioning (`version 0`) `.osa` files load as ORB automatically. `System` sets the type from config when mapping and **errors on a config/atlas mismatch** when loading.

**AKAZE (`AkazeExtractor`):**
- Wraps `cv::AKAZE` with M-LDB descriptors. By default `AKAZE.descriptorSize: 256` → **32-byte** descriptors that are byte-compatible with ORB, so the DBoW2 `FORB` class and the entire `ORBVocabulary` / Bag-of-Words pipeline are reused unchanged — an AKAZE vocabulary is just a differently trained vocabulary file of the **same type** and format (no vocabulary-type refactor). `descriptorSize: 0` selects the full 486-bit (61-byte) descriptor.
- **Scale model:** cv::AKAZE reports `keypoint.octave` in `[0, nOctaves)` with the feature scale doubling each octave, so the extractor is exposed as an `nOctaves`-level pyramid with `scaleFactor = 2.0`; the octave field is already a valid level index (no scale-space remapping). Sub-octave layers (`nOctaveLayers`) raise detection density but are collapsed onto their octave for the SLAM scale model.
- Honors the mono/stereo lapping-area split of `operator()` (returns `monoIndex`); caps to the strongest `ORBextractor.nFeatures` keypoints by response.
- YAML keys: `AKAZE.threshold` (default 0.001), `AKAZE.descriptorSize` (256), `AKAZE.nOctaves` (4), `AKAZE.nOctaveLayers` (4). They **must match** the values the AKAZE vocabulary was trained with. See `localization_service/example_akaze.yaml`.

**Run AKAZE:** pass the matching vocabulary (`Vocabulary/AKAZEvoc.txt`) and an AKAZE YAML:
```bash
./localization_service/localization_service_host \
    Vocabulary/AKAZEvoc.txt localization_service/example_akaze.yaml <source>
```

**Caveats:**
- The matcher thresholds `ORBmatcher::TH_LOW` / `TH_HIGH` (50 / 100) are tuned for 256-bit Hamming. AKAZE at 256-bit reuses them directly; a different `descriptorSize` may want different thresholds.
- SIFT (float / L2) is **not** implemented — it needs an L2 metric path in `ORBmatcher` and a float vocabulary class, which the 256-bit AKAZE approach deliberately avoids.

#### AKAZE vocabulary trainer (`localization_service/tools/akaze_vocab_trainer.cc`)

Trains a DBoW2 vocabulary from AKAZE-256 descriptors, in the same text format as `ORBvoc.txt`. It reuses `AkazeExtractor` (via the factory) so training and runtime configs cannot drift. Build with `cd build && ninja akaze_vocab_trainer`.

```bash
./localization_service/tools/akaze_vocab_trainer \
    --out Vocabulary/AKAZEvoc.txt -k 10 -L 6 \
    --descriptor-size 256 --threshold 0.001 --noctaves 4 --nlayers 4 --nfeatures 1500 \
    <imageDir> [<imageDir> ...]
```

- Image dirs are scanned **recursively** (`cv::glob`) for `.jpg/.jpeg/.png/.bmp`. Options: `--stride N` (subsample video frames), `--max-images N`, `--max-dim N` (downscale longer side to match the runtime processing width, e.g. 640).
- `-k`/`-L` set the tree (`10`/`6` ≈ 1M words, matching `ORBvoc.txt`; `-L 5` ≈ 100k for faster iteration).
- The AKAZE params **must match** the runtime YAML.
- All descriptors are held in RAM during training (`vector<vector<cv::Mat>>`); use `--stride`/`--max-images` to bound memory. The raw `.txt` is large (~145 MB for 1M words) — ship it compressed (`AKAZEvoc.txt.tar.gz`) like `ORBvoc.txt.tar.gz`.
- A general-purpose vocabulary benefits from diverse indoor+outdoor imagery (e.g. the Places365 validation set); a vocabulary trained only on the deployment environment is domain-specific and can bias relocalization tests run in that same space.

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

Executables are output to their respective source directories after compilation.

### `localization_service_host`

```
./localization_service/localization_service_host <vocab> <settings> <camera_source> [options]
```

**`camera_source`** values:

| Value | Behaviour |
|-------|-----------|
| `/dev/videoN` or `N` | V4L2 device |
| `http://…` / RTSP URL | MJPEG or RTSP stream |
| `none` | Frames pushed via `POST /api/frame` |
| `espnode` | Auto-discover ESP32 node via UDP broadcast (port 11211) |
| `espnode:<ip>` | Connect to ESP32 node at the given IP directly |

**Options** (all optional, any order):

| Flag | Default | Description |
|------|---------|-------------|
| `--localize` | off | Start in localization-only mode |
| `--map-id <n>` | `0` | Map index to activate on startup |
| `--port <n>` | `11142` | HTTP control server port |
| `--espnode-fps <n>` | `10` | Frame trigger rate for espnode source |

**Examples:**

```bash
# Mapping from USB webcam
./localization_service/localization_service_host \
    Vocabulary/ORBvoc.txt example.yaml /dev/video0

# Localization from a saved map
./localization_service/localization_service_host \
    Vocabulary/ORBvoc.txt example.yaml /dev/video0 \
    --localize --map-id 0

# ESP32 sensor node, auto-discover, 15 fps
./localization_service/localization_service_host \
    Vocabulary/ORBvoc.txt example.yaml espnode \
    --espnode-fps 15

# ESP32 sensor node, fixed IP, localization mode
./localization_service/localization_service_host \
    Vocabulary/ORBvoc.txt example.yaml espnode:192.168.1.42 \
    --localize --map-id 0 --espnode-fps 10
```

---

## ESP32-S3 Sensor Node (`Thirdparty/orblsammer_espnode/`)

An ESP32-S3 firmware + test client that acts as the physical camera and IMU front-end for the localization pipeline. Frames and IMU data arrive over a single persistent WiFi TCP connection; the host localization service feeds them into ORB-SLAM3 via `EspnodeSource`.

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
2. SD card mounted; reads SSID/password from `/config.txt` (lines 1 & 2) if present.
3. Camera init: PSRAM frame buffer, `CAMERA_GRAB_LATEST`, brightness +2, saturation −2, AGC on, gain ceiling ×64, AEC off, vertical flip on.
4. WiFi connected (station mode, sleep disabled).
5. TCP server started on **port 11212**.

**Main loop** (persistent single-client TCP session):
1. `handleSerial()` — processes serial commands non-blocking.
2. TCP client accepted; existing client replaced if a new one connects.
3. IMU update every iteration; readings accumulate in ring buffer (up to 128 × 6 floats).
4. UDP discovery broadcast every 100 iterations on **UDP port 11211**.
5. **TCP receive**: drain trigger bytes from host. Each `PACKET_TYPE_TRIGGER` (0x03) calls `sendCameraFrame()`, which sends IMAGE header + JPEG chunks then calls `tcpClient.flush()` before returning.
6. **TCP send**: if 50 ms have elapsed since last IMU flush, call `sendImuFrames()`. Write failures silently drop the batch without disconnecting.

### Wire Protocol

All packets share a **9-byte packed little-endian header**:

```
uint8_t  packet_type   // 0x01=IMAGE  0x02=IMU  0x03=TRIGGER (host→device, no header)
uint32_t frame_time    // millis() at send time
uint32_t total_size    // payload byte count
```

An IMU payload is `N × 24` bytes; each frame is six `float32` values:

```
roll, pitch, yaw  (radians, from accIntegral)
vx, vy, vz        (mm/s, GRAVITY = 9810 mm/s²)
```

**Flow control**: host sends at most one trigger at a time (`pendingTriggers` counter in `EspnodeSource`). The device calls `tcpClient.flush()` after each frame so the TCP send buffer is clear before the next IMU write.

### Network Ports

| Port | Protocol | Direction | Purpose |
|------|----------|-----------|---------|
| 11211 | UDP broadcast | ESP32 → LAN | IP auto-discovery |
| 11212 | TCP | bidirectional | Trigger (host→device), IMAGE + IMU stream (device→host) |

### Serial Commands

| Command | Action |
|---------|--------|
| `setwifi` | Interactive: enter SSID then password, write to `/config.txt`, reconnect WiFi |
| `status` | Print WiFi IP/RSSI, SD card state, TCP client count |
| `help` | List available commands |

### Host-side integration: `EspnodeSource`

**Files:** `localization_service/include/localization_service/espnode_source.h` and `src/espnode_source.cc`

- `EspnodeSource(ip, fps)` — `ip=""` for UDP auto-discovery, or a fixed IP.
- `discover(timeoutMs)` — binds UDP 11211, waits for the broadcast, returns the IP.
- `start(frameQueue, imuBuf)` — launches session thread (reconnects automatically on disconnect).
- `stop()` — signals shutdown and joins.

Two threads per connection: RX thread (blocking reads, `SO_RCVTIMEO=5s`) pushes packets onto a mutex-protected deque; session thread sends triggers and dispatches IMAGE → `IngestQueue` (via `cv::imdecode`) and IMU → `ImuBuffer`. `ImuBuffer` is drained once per frame in the main tracking loop.

### Test Client: `src/test_tcp_hud.py`

```bash
python src/test_tcp_hud.py       # 5 fps (default)
python src/test_tcp_hud.py 15    # 15 fps
```

Two-thread design (dedicated RX thread with 5s blocking timeout + main thread for triggers) prevents framing corruption from mid-payload socket timeouts. Displays frames in a window (`q` to quit) and prints IMU readings to the console.

### Build & Flash

```bash
pio run -t upload      # compile and flash
pio device monitor     # serial output at 115200 baud
```

WiFi credentials: use the `setwifi` serial command, or place `/config.txt` on the SD card (line 1 = SSID, line 2 = password).
