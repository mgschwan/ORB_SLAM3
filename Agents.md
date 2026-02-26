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
├── Examples/                - Active examples (only monocular built by modified CMake)
│   ├── Monocular/
│   │   ├── mono_tum_mgschwan.cc       - [CUSTOM] TUM dataset with localization mode
│   │   ├── mono_tum_mgschwan          - [BUILT EXECUTABLE]
│   │   ├── remote_tum.cc              - [CUSTOM] Live camera stream with localization mode
│   │   ├── remote_tum                 - [BUILT EXECUTABLE]
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
│   └── Sophus/              - Lie group math library (SE3, SO3, Sim3)
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
- **Map Management:** Added `Atlas::SwitchToMap(int idx)` to allow programmatically switching between different maps in the Atlas.

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

### 3. New Executables (Examples/Monocular)
- **`mono_tum_mgschwan.cc`**: 
    - Supports an optional `localize_only` command-line argument.
    - Specifically designed to work with TUM dataset format but with the improved localization logic.
- **`remote_tum.cc`**:
    - Uses `cv::VideoCapture` to process streams from a URL (e.g., IP camera, MJPEG stream).
    - Supports localization mode and manual map selection via `map_id`.
    - Automatically calls `ForceRelocalization()` when starting in localization mode.

### 4. Configuration & Settings
- **Loop Closing Toggle:** Added support for a `loopClosing` flag in the YAML settings file to enable/disable the Loop Closing thread.
- **Image Scaling:** Improved handling of image scaling in the examples to match the SLAM system's expectations.

## Technical Details

### Localization Mode Activation
To activate the improved localization mode in your code:
```cpp
SLAM.ActivateLocalizationMode();
SLAM.GetAtlas()->SwitchToMap(map_id); // Optional: Switch to a specific saved map
SLAM.ForceRelocalization();          // Force the system to start searching in the map
```

### Tracking Logic Change (`Tracking.cc`)
The core change preventing new map creation is in `Tracking::Track()`:
```cpp
// Original logic would create a new map if lost
if(mState==LOST && !mbOnlyTracking) // Added !mbOnlyTracking check
{
    // ...
    CreateMapInAtlas();
}
```

### Build Requirements
- OpenCV
- Eigen3
- Sophus (included in Thirdparty)
- DBoW2 & g2o (included in Thirdparty)
- (Optional) Pangolin

## Usage
The new executables can be found in the `build` directory after compilation.
Example for `remote_tum`:
```bash
./remote_tum path_to_vocabulary path_to_settings camera_url [localize_only] [map_id]
```
