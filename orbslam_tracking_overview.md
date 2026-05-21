# ORB-SLAM3 Tracking::Track() — Deep Walkthrough

This document explains what happens inside `Tracking::Track()` (src/Tracking.cc:1794) per frame, and how every major case branches. It is written for human readers who want to understand the full system, particularly for debugging the forced-pose injection feature.

---

## 0. When is Track() called?

`Track()` is a private method called from three public entry points in response to a new image:
- `GrabImageMonocular()` — for a single camera (our use case)
- `GrabImageStereo()` — for stereo cameras
- `GrabImageRGBD()` — for RGB-D cameras

The caller extracts ORB features from the image and fills in `mCurrentFrame` before calling `Track()`.

---

## 1. State Machine Overview

`Tracking` is a state machine. `mState` holds the current state:

| State | Meaning |
|---|---|
| `NO_IMAGES_YET` | First frame not yet processed, or just after `CreateMapInAtlas()` reset. |
| `NOT_INITIALIZED` | Sensor active, initial map not yet created. |
| `OK` | Actively tracking, pose is trusted. |
| `RECENTLY_LOST` | Tracking just failed. System tries recovery for a short grace period (~3–5 s). |
| `LOST` | Extended failure. System will either try hard relocalization or create a new map. |

`mLastProcessedState` is a snapshot of the state before this frame, used by the FrameDrawer for display.

---

## 2. Pre-flight Checks (Lines 1797–1906)

Before doing any tracking work, `Track()` runs several guards:

**Step-by-step debug mode:** If `bStepByStep` is set, the function waits for a manual trigger. Not relevant in normal use.

**Bad IMU flag:** If LocalMapping set `mbBadImu`, the active map is reset immediately.

**Timestamp sanity:**
- If the current frame is *older* than the last frame (clock went backwards), the IMU queue is cleared and a new map is created.
- If there is a gap of more than 1 second (for IMU sensors), the system resets or creates a new map, depending on whether the IMU has been initialized.

**IMU Preintegration:** If the sensor includes an IMU, `PreintegrateIMU()` accumulates all IMU measurements that arrived since the last frame into a rotation/velocity/position delta.

**Map change detection:** The code checks `pCurrentMap->GetMapChangeIndex()` against a saved value. If the map was updated by LoopClosing (e.g. after a loop closure), `mbMapUpdated` is set to `true`. This flag later influences which optimizer variant is used.

**State transition on first frame:**
- If the map already has keyframes when tracking starts (e.g. loaded from disk), the state jumps to `RECENTLY_LOST` and the lost-timer is started. The system will try to relocalize immediately.
- If the map is empty, the state is set to `NOT_INITIALIZED`.

---

## 3. Initialization Phase (State = NOT_INITIALIZED)

### Stereo / RGB-D Initialization (`StereoInitialization`)

With a stereo or RGB-D camera, a single frame is sufficient to build the initial map because depth is directly available:

1. Requires ≥500 feature points in the frame (a high threshold because depth quality is checked).
2. Creates two `KeyFrame` objects (the initial one) — actually just one, since there is no second frame needed.
3. Directly unprojects each feature point using the depth value into a 3D world point.
4. These `MapPoint`s go straight into the map at their measured 3D positions.
5. State is set to `OK`.

### Monocular Initialization (`MonocularInitialization`)

With only one camera there is no depth information. Two frames are needed to triangulate points. The process takes *two calls*:

**Phase 1 (first call while `mbReadyToInitializate == false`):**
- If the current frame has ≥100 ORB keypoints, it is saved as `mInitialFrame`.
- `mbReadyToInitializate` is set to `true`.
- If a forced pose was provided at this moment (via `SetNextFramePose`), it is saved as `mInitialFrameForcedPose` and `mbInitialFrameHasForcedPose` is set — used in the metric-pose path below.
- Returns without creating anything.

**Phase 2 (second call while `mbReadyToInitializate == true`):**
- If the current frame also has ≥100 points, ORB matcher tries to find at least 100 corresponding features between `mInitialFrame` and `mCurrentFrame`.
- `mpCamera->ReconstructWithTwoViews()` is called. This runs RANSAC with both a homography model (for planar scenes) and a fundamental matrix model (for general motion) and picks the best solution. The output is a relative pose `Tcw` (transform from world — defined as the initial frame origin — to the current camera) and a set of triangulated 3D points `mvIniP3D`.
- **Normal path:** `mInitialFrame` is set at the world origin (identity pose), `mCurrentFrame` is set to `Tcw`.
- **Forced-pose path:** If *both* frames had external poses injected (`mbInitialFrameHasForcedPose && mbHasForcedPose`), the reconstructed points are scaled and rotated so they live in real metric world coordinates matching those two external poses. `mbForcedPoseInitialization` is set to skip the median-depth rescaling in the next step.
- `CreateInitialMapMonocular()` is called.

### `CreateInitialMapMonocular()`

This turns the initialization frames into actual map structures:

1. Creates two `KeyFrame` objects from `mInitialFrame` and `mCurrentFrame`.
2. Inserts them into the Atlas.
3. For every successfully triangulated point in `mvIniP3D`, creates a `MapPoint` with world position, links it to both keyframes as an observation.
4. Calls `Optimizer::GlobalBundleAdjustment()` (20 iterations) to refine both keyframe poses and all map point positions simultaneously.
5. **Scale normalization (monocular only):** Monocular reconstruction produces a scene with arbitrary scale. The median scene depth is computed and all translations and map point positions are divided by it, setting the scale so that the median depth equals 1.0 metre. **This step is skipped when `mbForcedPoseInitialization` is true** because the forced poses already define metric scale.
6. Inserts both keyframes into the LocalMapper queue.
7. Sets `mState = OK`.

---

## 4. Normal Tracking (State = OK or RECENTLY_LOST)

Once the map exists, each frame goes through up to three stages:

```
Stage A: Pose Prediction  →  Stage B: TrackLocalMap  →  Stage C: KeyFrame Decision
```

### Stage A — Pose Prediction

The goal is to get a rough estimate of where the camera is. Exact accuracy is less important here; `TrackLocalMap` will refine it. Several strategies exist; the system picks one depending on what information is available.

#### A1 — Forced External Pose (`mbHasForcedPose == true`, state OK, mapping mode)

If `SetNextFramePose(Tcw)` was called before this frame, the current frame is immediately assigned that pose. `bOK = true`. The pose-prediction stage is completely skipped. `bForcedPoseFrame = true` and `savedForcedPose` is stored so that after `TrackLocalMap` runs (which will optimize the pose and possibly overwrite it), the original forced pose can be restored.

#### A2 — Track with Motion Model (`TrackWithMotionModel`)

This is the normal fast path when the camera is moving smoothly.

**Prerequisites:** `mbVelocity` is true (we have a velocity estimate from at least the previous two frames) and we are not right after a relocalization.

**How velocity is computed:** At the end of each successful frame, the velocity is computed as:
```
mVelocity = Tcw_current * Twc_last   (i.e. the relative pose change frame-to-frame)
```

**TrackWithMotionModel steps:**
1. `UpdateLastFrame()` — refreshes the last frame's pose from its reference keyframe's latest (possibly optimized) position. For stereo/RGB-D in localization mode, also creates temporary "visual odometry" map points by unprojecting depth features into 3D — these are *not* added to the persistent map (they go into `mlpTemporalPoints` and are deleted at the end of the frame).
2. **IMU branch:** If the IMU is initialized and enough frames have passed since the last relocalization, `PredictStateIMU()` is called instead of the visual motion model — see A3 below.
3. **Visual motion model branch:** The predicted pose is: `Tcw_predicted = mVelocity * Tcw_last`.
4. `ORBmatcher::SearchByProjection()` — the map points from the last frame are projected into the current frame (using the predicted pose) to find matches in a small search window. If too few matches are found (<20), the search window is doubled.
5. `Optimizer::PoseOptimization()` — a G2O graph optimization that adjusts only the camera pose (6-DOF), keeping all map points fixed. Uses a Huber robust cost function. Outlier map points are flagged.
6. Returns `true` if ≥10 map points (non-VO) or ≥20 total matches pass after outlier removal. For IMU sensors, always returns `true` since IMU provides a fallback.

If the motion model fails, the system falls back to A4.

#### A3 — IMU Pose Prediction (`PredictStateIMU`)

Called inside `TrackWithMotionModel` (if IMU initialized) or standalone during `RECENTLY_LOST` recovery.

IMU preintegration uses the accumulated gyroscope and accelerometer measurements to predict:
- `Rwb2` — new body rotation (integrated from last known rotation via gyro delta)
- `twb2` — new body position (integrated from last known position via velocity + accelerometer, accounting for gravity)
- `Vwb2` — new body velocity

Two variants:
- If `mbMapUpdated` (LoopClosing just updated the map), integrate from the last *keyframe* pose.
- Otherwise, integrate from the last *frame* pose (more frequent, lower latency).

This prediction is deterministic and does not require feature matching, so it always succeeds. The motion model after this is treated as a reliable prior.

#### A4 — Track Against Reference KeyFrame (`TrackReferenceKeyFrame`)

Used when: no velocity is available, just after a relocalization, or as a fallback when the motion model fails.

**Steps:**
1. Compute BoW vector for current frame (decomposes ORB descriptors into the vocabulary tree representation).
2. Match ORB features between the current frame and `mpReferenceKF` using `SearchByBoW` — faster than brute-force because BoW groups features by vocabulary node.
3. If <15 matches, fail.
4. Set the current frame pose to the last frame's pose (as initial guess).
5. `Optimizer::PoseOptimization()` — same as step 5 above.
6. Returns `true` if ≥10 inlier map-point matches remain.

#### A5 — Relocalization (`Relocalization()`)

Used when state is `RECENTLY_LOST` (no IMU), or `LOST` in localization mode, or when `mbAllowMapCreation == false` and state is `LOST`.

**Steps:**
1. Compute BoW vector for current frame.
2. Query `KeyFrameDatabase::DetectRelocalizationCandidates()` — uses inverse-index BoW lookup to find keyframes that share visual words with the current frame. The score is bounded to the active map only (post-fix from this repo's modifications).
3. For each candidate keyframe: run `SearchByBoW` to find individual feature matches.
4. For candidates with ≥15 matches, create an `MLPnPsolver` (Maximum Likelihood Perspective-n-Point). RANSAC parameters: confidence 0.99, min inliers 10, max iterations 300, min set 6.
5. Iteratively run 5 RANSAC iterations on each surviving candidate until a pose is found.
6. If a rough pose is found, run `PoseOptimization`. If inliers ≥50 → **success**.
7. If inliers < 50 but ≥10, try an additional wider `SearchByProjection` (window 10px, scale factor 100), then re-optimize. Can get to 50 inliers this way.
8. If still 30–50 inliers: try a narrow `SearchByProjection` (window 3px, scale factor 64) and a final optimization pass.
9. If the pose accumulates ≥50 inliers, `bMatch = true`, `mnLastRelocFrameId` is recorded, and the winning candidate becomes the new `mpReferenceKF`.

A successful relocalization brings the system back from `LOST` to `OK` via `TrackLocalMap` in Stage B.

---

## 5. State Transitions During Stage A

After Stage A attempts, `bOK` and the current `mState` determine the next state:

```
If state was OK and Stage A succeeded   → stay OK
If state was OK and Stage A failed:
   - Recent reloc + IMU              → LOST (hard)
   - ≥10 keyframes in map            → RECENTLY_LOST (soft grace period)
   - Otherwise                       → LOST (hard)

If state was RECENTLY_LOST (no IMU):
   - Relocalization succeeded         → continue to Stage B
   - Relocalization failed, <3s      → stay RECENTLY_LOST, bOK=false
   - Relocalization failed, ≥3s      → LOST

If state was RECENTLY_LOST (with IMU):
   - IMU prediction available         → continue (PredictStateIMU), bOK=true
   - IMU not yet initialized          → bOK=false
   - Time since lost > time_recently_lost (5s default) → LOST

If state was LOST (mapping mode, mbAllowMapCreation=true):
   - Fall through to CreateMapInAtlas at the bottom of Track()

If state was LOST (mapping mode, mbAllowMapCreation=false):
   - Relocalization is called every frame
   - If fails → return early (frame is discarded)

If state was LOST (localization mode, mbOnlyTracking=true):
   - Relocalization is called every frame
```

---

## 6. Stage B — TrackLocalMap

`TrackLocalMap()` is the refinement step. It runs only when Stage A produced at least a rough pose estimate (`bOK == true`) and `mbVO == false` (or we are in mapping mode).

### What is the "local map"?

The local map is a subset of the global map: keyframes and their map points that are spatially close to the current camera position. It is rebuilt every frame by `UpdateLocalMap()`:

**`UpdateLocalKeyFrames()`:**
- Counts how many currently-tracked map points each keyframe has observed (the "vote").
- The keyframe with the most votes becomes the new `mpReferenceKF`.
- Includes the top-voted KFs, plus their best covisibility neighbors, their spanning-tree children, and their spanning-tree parents.
- Cap: up to 80 local keyframes.
- For IMU sensors: also includes the 20 most recent keyframes in the temporal chain.
- **Forced-pose edge case (modified):** If the current frame has no map-point associations yet (e.g. very first frame with a forced pose), the keyframe counter is empty. The code seeds it from `mpReferenceKF` and the spatially nearest keyframe, so that `SearchLocalPoints` can still find candidates to project.

**`UpdateLocalPoints()`:**
- Collects all map points seen by any local keyframe into `mvpLocalMapPoints`.

### `SearchLocalPoints()`

Projects every map point in `mvpLocalMapPoints` into the current frame (using the current estimated pose). For each point that falls within the image frustum, attempts to find a matching ORB feature:
- Already-matched points (from Stage A) are skipped (their visible/found counters are updated).
- Bad points are removed from `mCurrentFrame.mvpMapPoints`.
- Remaining local map points are tested with `isInFrustum()` and, if visible, matched with `SearchByProjection`.
- The search radius (`th`) depends on the sensor type, whether the camera just relocated, and whether it is in a lost state (wider radius when uncertain).

### Pose Optimization in TrackLocalMap

After adding more matches from the local map, the pose is re-optimized with more constraints:

- **No IMU:** `Optimizer::PoseOptimization()` — standard visual-only 6-DOF optimization.
- **IMU initialized, recent relocalization:** `Optimizer::PoseOptimization()` — fall back to visual-only while IMU resets.
- **IMU initialized, map just updated (LoopClosing ran):** `Optimizer::PoseInertialOptimizationLastKeyFrame()` — uses IMU preintegration constraint anchored at the last *keyframe*.
- **IMU initialized, normal operation:** `Optimizer::PoseInertialOptimizationLastFrame()` — uses IMU preintegration constraint anchored at the last *frame* (tighter, lower-latency).

### Success threshold (TrackLocalMap return value)

`mnMatchesInliers` counts all map points with `Observations() > 0` that survived PoseOptimization as inliers.

| Sensor | Threshold |
|---|---|
| Pure monocular / stereo / RGB-D | ≥30 inliers |
| IMU monocular (IMU initialized) | ≥15 inliers |
| IMU monocular (IMU not yet initialized) | ≥50 inliers |
| IMU stereo / RGB-D | ≥15 inliers |
| Any, just after relocalization | ≥50 inliers (stricter grace period) |
| Any, state RECENTLY_LOST | ≥10 inliers (lenient recovery) |

### Forced-pose override after TrackLocalMap

When `bForcedPoseFrame == true`, after `TrackLocalMap` runs:
- The frame's pose is **restored** to `savedForcedPose` (overwriting whatever PoseOptimization computed).
- `bOK` is forced to `true`.
- All outlier flags on map points are cleared (`mvbOutlier[i] = false`) because the outlier flags were computed relative to the optimizer's (now discarded) pose, not the trusted external one.

---

## 7. Motion Model Update

If `bOK` is true (or state is `RECENTLY_LOST`), the velocity model is updated:

```cpp
mVelocity = Tcw_current * Twc_last
```

This is the relative camera motion from the previous frame to the current one, expressed as a `SE3f` transform. Used as the prediction for the next frame in Stage A2.

If the current or last frame pose is not set (e.g. during the brief period after initialization or after a forced pose that hasn't been verified yet), `mbVelocity` is set to false and Stage A will fall back to TrackReferenceKeyFrame next frame.

---

## 8. Stage C — KeyFrame Decision and Creation

### When is a new keyframe inserted?

`NeedNewKeyFrame()` returns `true` when:

- **`mbOnlyTracking == true` (localization mode):** Always returns `false`. No new keyframes are ever created in localization mode (unless the IMU is not yet initialized — IMU-monocular needs KFs to initialize IMU).
- **LocalMapper is stopped or busy:** Returns `false` (loop closure in progress).
- **Within the relocalization grace period** (`mnLastRelocFrameId + mMaxFrames`): Returns `false`.

If those guards pass, the conditions are:
- `c1a`: More than `mMaxFrames` (≈ 1/fps × fps = 1 second) since the last keyframe.
- `c1b`: More than `mMinFrames` (0) since the last keyframe AND LocalMapper is idle.
- `c1c`: (Stereo/RGB-D only) Fewer than 25% of reference-KF matches tracked, or too few close points.
- `c2`: Fewer inliers than `thRefRatio × nRefMatches`, or need close points, AND still at least 15 inliers.
  - `thRefRatio` is 0.9 for monocular (demanding), 0.75 for stereo/fisheye.
- `c3`: (IMU) ≥0.5 s since last keyframe.
- `c4`: (IMU monocular) Inliers between 15–75 or state is RECENTLY_LOST.

Condition: `(c1a || c1b || c1c) && c2`, or `c3`, or `c4`.

**Forced-pose override:** If `bForcedPoseFrame == true` and `NeedNewKeyFrame()` returned `false`, a keyframe is inserted anyway if LocalMapper can accept it. This ensures that when an external pose is provided, triangulation can proceed even if the match-ratio criterion would normally suppress a new keyframe.

### What happens in `CreateNewKeyFrame(bool bForcedPose)`

1. Creates a `KeyFrame` from `mCurrentFrame` and stores `bForcedPose` in `pKF->mbForcedPose`.
2. Makes this keyframe the new `mpReferenceKF`.
3. Links it into the temporal chain (`mPrevKF` / `mNextKF`) and creates a fresh IMU preintegration accumulator.
4. **For stereo/RGB-D only** — creates additional map points directly in this function:
   - Collects all features with a valid depth reading.
   - Sorts by depth (closest first).
   - For features that do not yet have a map point (or have a bad one), unprojects the depth to 3D and creates a new persistent `MapPoint`.
   - Stops after 100 close points (within `mThDepth`) or when all points with depth < threshold are exhausted.
   - These map points are *permanent* (unlike VO points). They are added to the Atlas immediately.
5. **For monocular** — does NOT create map points here. New monocular map points are triangulated by `LocalMapping` in the background.
6. Sends the keyframe to `LocalMapper` via `InsertKeyFrame()`.

### Where do monocular map points come from?

In monocular mode, map points cannot be created from a single frame. They are triangulated by the `LocalMapping` thread in the background:

1. When a new keyframe is inserted, `LocalMapping::CreateNewMapPoints()` runs.
2. For each neighbor keyframe in the covisibility graph, pairs of unmatched ORB features are matched across the two keyframes.
3. For each matched pair, the two bearing rays are intersected (triangulated) to get a 3D position.
4. Validity checks: positive depth in both cameras, reprojection error < threshold, parallax > minimum angle.
5. A new `MapPoint` is created and added to the Atlas.
6. `LocalMapping::SearchInNeighbors()` fuses duplicate map points (same 3D point seen in more keyframes than recorded).

So in monocular mode, **map points exist only as a result of the LocalMapper observing the same scene point from at least two keyframes**. The Tracking thread's job is to *find existing map points* and use them for pose estimation. It does not triangulate itself (except during the two-frame initialization).

---

## 9. Localization-Only Mode (`mbOnlyTracking == true`)

This mode is activated by `SLAM.ActivateLocalizationMode()`. The local mapper thread continues to run but `NeedNewKeyFrame()` always returns `false`, so it never receives new keyframes and the map is frozen.

### The `mbVO` flag

`mbVO` ("Visual Odometry") is a flag that says "the current tracking is based on temporary points, not the persistent map." It is set to `true` by `TrackWithMotionModel` when fewer than 10 of the matches are to permanent map points.

**Localization mode state machine during Stage A:**

```
State = LOST:
   → Relocalization() every frame until it succeeds
   → If it succeeds, state goes to OK via TrackLocalMap

State = OK or RECENTLY_LOST, mbVO = false:
   → Normal: TrackWithMotionModel (if velocity) or TrackReferenceKeyFrame
   → If both fail → mbVO = true (MODIFIED version; original: set mbVO=true but keeps OK)

State = OK or RECENTLY_LOST, mbVO = true:
   → First tries Relocalization(). If it succeeds → mbVO=false, bOK=true (back to map tracking)
   → If relocalization fails:
       → Tries TrackWithMotionModel (pure odometry, uses temporary points)
       → If motion model succeeds → bOK=true (drifting VO)
       → If motion model fails → LOST (hard)
```

**Visual odometry points:** In `UpdateLastFrame()`, for stereo/RGB-D in localization mode, temporary map points are created by unprojecting depth features. These allow the camera pose to be tracked relative to the last frame even when it cannot see any persistent map points. They are stored in `mlpTemporalPoints` and deleted at the end of each frame.

**TrackLocalMap in localization mode:** Only runs if `mbVO == false`. When in VO mode, there are no permanent local map points to project, so this step is skipped.

---

## 10. The Forced Pose Path (`mbHasForcedPose`)

This is a custom addition to the codebase. It allows an external caller to inject a known camera pose, bypassing the visual tracking for that frame.

### How it is set

`Tracking::SetNextFramePose(Sophus::SE3f Tcw)` sets `mbHasForcedPose = true` and saves `mForcedPose`. In `localization_service_host.cc`, this is called when a frame from the ingest queue has `hasPose == true`:

```cpp
if (ingest.hasPose)
    slam.SetNextFramePose(ingest.Tcw);
```

### What happens in Track() when `mbHasForcedPose` is set

The forced pose path **only activates when state is OK and `mbOnlyTracking == false`** (mapping mode):

```cpp
if (mbHasForcedPose) {
    savedForcedPose  = mForcedPose;
    bForcedPoseFrame = true;
    mCurrentFrame.SetPose(mForcedPose);
    mbHasForcedPose  = false;
    bOK = true;
}
```

The flag is consumed immediately. The frame's pose is set. Stage A is skipped.

Then `TrackLocalMap` runs — this is *intentional*. The purpose is to associate map points with the current frame at the provided pose, which gives the triangulation system something to work with.

After `TrackLocalMap` returns, the forced pose is **restored** (overwriting what PoseOptimization computed), and all outlier flags are cleared.

The keyframe insertion logic may then create a new keyframe tagged with `mbForcedPose = true`.

### Known problems / limitations with the current forced pose implementation

The forced pose path has several constraints that can cause it to fail silently:

1. **It only activates in state OK.** If the system is in `NOT_INITIALIZED`, `RECENTLY_LOST`, or `LOST`, the forced pose field is read but has no effect on pose selection — the normal loss recovery logic runs instead. The forced pose injected during initialization is saved to `mInitialFrameForcedPose` for Phase 2 handling, but the current-frame forced pose is discarded if state is not OK.

2. **It only activates in mapping mode (`mbOnlyTracking == false`).** The localization mode branch (`else` at line 2059) has no code to check `mbHasForcedPose`. If the system is in localization mode and a forced pose is provided, it is silently ignored.

3. **Initialization with forced poses requires both frames to have poses.** During monocular initialization (Phase 2), the metric forced-pose path only triggers if `mbInitialFrameHasForcedPose && mbHasForcedPose` — both the initial frame AND the current frame must have poses. If only the second frame has a pose, the standard (up-to-scale) path runs.

4. **TrackLocalMap's PoseOptimization overwrites the forced pose.** The code restores it after the fact (line 2150), but the outlier flagging done during optimization reflects the *optimizer's* pose, not the forced one. The current fix clears all outlier flags, which means genuinely bad matches are kept and may confuse the LocalMapper.

5. **The local map may be empty on the first forced-pose frame.** If the local keyframe set is empty or wrong (e.g. the forced pose places the camera far from `mpReferenceKF`), `SearchLocalPoints` finds nothing to project and `TrackLocalMap` will fail its inlier count threshold even though the forced pose is valid. The `UpdateLocalKeyFrames` modification (finding the spatially nearest KF) is intended to mitigate this but requires that the map already has KFs near the forced position.

6. **`mbVelocity` becomes wrong after forced frames.** The velocity is recomputed as `Tcw_current * Twc_last` at the end of every frame. If the forced pose represents a large jump, the computed velocity is nonsensical and the next frame's motion model will predict a wrong position. The frame after a forced frame should ideally not rely on the motion model.

---

## 11. End-of-Frame Bookkeeping

If `bOK` is true (or state is RECENTLY_LOST):

1. **Motion model update:** `mVelocity = Tcw_current * Twc_last`.
2. **Clean VO matches:** Map points with `Observations() < 1` (temporary VO points that leaked into `mvpMapPoints`) are removed from the current frame.
3. **Delete temporal points:** All items in `mlpTemporalPoints` (the stereo VO points from `UpdateLastFrame`) are deleted.
4. **KeyFrame decision and creation** (described in Section 8).
5. **Outlier removal:** Any map point still flagged as an outlier after TrackLocalMap is removed from `mCurrentFrame.mvpMapPoints`. These are "on probation" — they were allowed through to the keyframe (so LocalMapping can do a final BA decision) but are not used for the next frame's pose prediction.

If state is `LOST` and not in localization mode:
- If ≤10 keyframes in map → `ResetActiveMap()` (start fresh from scratch).
- If IMU is not yet initialized → `ResetActiveMap()`.
- If `mbAllowMapCreation` → `CreateMapInAtlas()` (freeze current map, start a new one).

**Trajectory storage:** The current frame's pose relative to its reference keyframe (`Tcr = Tcw * Twc_ref`) is stored in `mlRelativeFramePoses`. This accumulates the full trajectory for later export (e.g. `SaveKeyFrameTrajectoryTUM`).

---

## 12. Data Flow Summary Diagram

```
New Image Arrives
       │
       ▼
ORB Feature Extraction
       │
       ▼
       ┌─────────────────────────────────────────────────────────────────┐
       │                     Tracking::Track()                           │
       │                                                                 │
       │  Pre-flight checks (timestamp, IMU preintegration, map updates) │
       │                                                                 │
       │  ┌─────────────────────────────────────────────────────────┐   │
       │  │  State = NOT_INITIALIZED                                  │   │
       │  │  → StereoInit or MonocularInit (2-frame process)         │   │
       │  │  → On success: create KeyFrames, triangulate, BA         │   │
       │  │  → State = OK                                            │   │
       │  └─────────────────────────────────────────────────────────┘   │
       │                                                                 │
       │  ┌─────────────────────────────────────────────────────────┐   │
       │  │  State = OK / RECENTLY_LOST / LOST                        │   │
       │  │                                                           │   │
       │  │  STAGE A: Pose Prediction                                 │   │
       │  │  ┌──────────────────────────────────────────────────┐    │   │
       │  │  │ mapping mode, state OK:                           │    │   │
       │  │  │   forced pose? → inject directly                  │    │   │
       │  │  │   else:                                           │    │   │
       │  │  │     velocity? → TrackWithMotionModel              │    │   │
       │  │  │       IMU initialized? → PredictStateIMU          │    │   │
       │  │  │       else: visual velocity model                 │    │   │
       │  │  │     (fallback) → TrackReferenceKeyFrame           │    │   │
       │  │  │                                                   │    │   │
       │  │  │ mapping mode, state RECENTLY_LOST:                │    │   │
       │  │  │   IMU available? → PredictStateIMU               │    │   │
       │  │  │   else: → Relocalization                         │    │   │
       │  │  │                                                   │    │   │
       │  │  │ mapping mode, state LOST:                         │    │   │
       │  │  │   mbAllowMapCreation=false: → Relocalization      │    │   │
       │  │  │   mbAllowMapCreation=true:  → CreateMapInAtlas    │    │   │
       │  │  │                                                   │    │   │
       │  │  │ localization mode, state LOST:                    │    │   │
       │  │  │   → Relocalization (every frame until success)    │    │   │
       │  │  │                                                   │    │   │
       │  │  │ localization mode, state OK, mbVO=false:          │    │   │
       │  │  │   → TrackWithMotionModel or TrackReferenceKF      │    │   │
       │  │  │                                                   │    │   │
       │  │  │ localization mode, state OK, mbVO=true:           │    │   │
       │  │  │   → Relocalization first                          │    │   │
       │  │  │   fail? → TrackWithMotionModel (VO drifting)      │    │   │
       │  │  └──────────────────────────────────────────────────┘    │   │
       │  │                                                           │   │
       │  │  STAGE B: TrackLocalMap (if bOK)                          │   │
       │  │    UpdateLocalKeyFrames → find up to 80 nearby KFs        │   │
       │  │    UpdateLocalPoints   → collect their MapPoints           │   │
       │  │    SearchLocalPoints   → project local MPs, match ORB      │   │
       │  │    PoseOptimization    → refine pose with all matches       │   │
       │  │    (forced pose frame: restore external pose after BA)      │   │
       │  │    Count inliers → success/fail                            │   │
       │  │                                                           │   │
       │  │  Update state (OK / RECENTLY_LOST / LOST)                  │   │
       │  │  Update velocity model                                     │   │
       │  │                                                           │   │
       │  │  STAGE C: KeyFrame Decision (if bOK)                       │   │
       │  │    NeedNewKeyFrame? (loc mode → always no)                 │   │
       │  │    forced pose frame? → force KF if mapper accepts          │   │
       │  │    CreateNewKeyFrame:                                      │   │
       │  │      stereo/RGB-D: unproject depth → new MapPoints         │   │
       │  │      monocular: KF queued, LocalMapper triangulates later  │   │
       │  └─────────────────────────────────────────────────────────┘   │
       │                                                                 │
       └─────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
                              Pose returned to caller
```

---

## 13. Key Variable Reference

| Variable | Type | Meaning |
|---|---|---|
| `mState` | `eTrackingState` | Current tracking state |
| `mbOnlyTracking` | `bool` | Localization mode (no new KFs or MPs) |
| `mbAllowMapCreation` | `bool` | Whether LOST triggers a new map |
| `mbVO` | `bool` | Current tracking is VO-only (no map matches) |
| `mbVelocity` | `bool` | Velocity model is valid |
| `mVelocity` | `SE3f` | Frame-to-frame relative motion (prediction) |
| `mpReferenceKF` | `KeyFrame*` | The KF with most shared map points with current frame |
| `mvpLocalKeyFrames` | `vector<KF*>` | Spatial neighborhood of KFs |
| `mvpLocalMapPoints` | `vector<MP*>` | All MPs seen by local KFs |
| `mnMatchesInliers` | `int` | Inlier map-point matches after TrackLocalMap |
| `mnLastRelocFrameId` | `uint` | Frame ID when last relocalization succeeded |
| `mTimeStampLost` | `double` | Timestamp when tracking was first lost |
| `mbHasForcedPose` | `bool` | External pose waiting to be consumed |
| `mForcedPose` | `SE3f` | The external pose (Tcw, camera-to-world inverse) |
| `bForcedPoseFrame` | `bool` | (local) This frame used a forced pose |
| `savedForcedPose` | `SE3f` | (local) Copy of forced pose for post-TLM restore |
| `mlpTemporalPoints` | `list<MP*>` | Temporary stereo/VO points deleted each frame |
