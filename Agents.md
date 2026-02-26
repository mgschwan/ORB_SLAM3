# Project Summary - ORB_SLAM3 Modified

This project is a modified version of ORB-SLAM3. The primary goals of the modifications were to remove the mandatory dependency on Pangolin (for visualization) and to improve the "localization only" mode for robust use in pre-mapped environments.

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
