/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;

class Tracking
{  

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq=std::string());

    ~Tracking();

    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);

    void GrabImuData(const IMU::Point &imuMeasurement);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetStepByStep(bool bSet);
    bool GetStepByStep();

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);
    // cameraType: GeometricCamera::CAM_PINHOLE (0) or GeometricCamera::CAM_FISHEYE (1)
    void ChangeCalibration(const cv::Mat &K, const cv::Mat &DistCoef,
                           unsigned int cameraType = GeometricCamera::CAM_PINHOLE);
    unsigned int GetCameraType() { return mpCamera ? mpCamera->GetType() : GeometricCamera::CAM_PINHOLE; }

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    void CreateMapInAtlas();
    //std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();

    //DEBUG
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder="");
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap);

    float GetImageScale();

    // Feature/descriptor type selected from the `Feature.type` YAML key.
    FeatureType GetFeatureType() const { return mFeatureType; }

    // Target-resolution downscaling: returns the configured internal processing
    // resolution (0,0 if disabled). See ParseCamParamFile / GrabImageMonocular.
    void GetTargetSize(int &w, int &h);

    void SetAtlas(Atlas* pAtlas) { mpAtlas = pAtlas; }

    bool ForceRelocalization() { mState = LOST; return true; }

    void InformMapSwitch() {
        mState = RECENTLY_LOST;
        mTimeStampLost = mCurrentFrame.mTimeStamp;
        mbVelocity = false;
        mbVO = false;
        
        // Clear local mapping variables
        mvpLocalKeyFrames.clear();
        mvpLocalMapPoints.clear();
        mpReferenceKF = nullptr;
        mpLastKeyFrame = nullptr;
    }

#ifdef REGISTER_LOOP
    void RequestStop();
    bool isStopped();
    void Release();
    bool stopRequested();
#endif

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        RECENTLY_LOST=3,
        LOST=4,
        OK_KLT=5
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;

    cv::Mat mImGray;
    cv::Mat mInitImGray;   // grayscale image of the monocular initialization frame (debug)

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<Sophus::SE3f> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // frames with estimated pose
    int mTrackedFr;
    bool mbStep;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    // When false, the tracker will never call CreateMapInAtlas() on loss of tracking.
    // Defaults to true (original upstream behaviour). Set to false to keep the
    // system locked to the current map even in full mapping mode.
    bool mbAllowMapCreation;
    void SetAllowMapCreation(bool allow) { mbAllowMapCreation = allow; }

    // Inject an external pose for the next frame.  When set, Tracking skips
    // its feature-based estimator and PoseOptimization for that frame.
    // TrackLocalMap still runs for map-point association.
    // Thread-safe: called from the ingest path before TrackMonocular.
    void SetNextFramePose(const Sophus::SE3f& Tcw);

    // Monocular initialization path used when both the initial frame and the
    // current frame carry externally provided poses.  Defined in ForcedPoseInit.cc.
    void ForcedPoseMonocularInit(int nmatches);

    bool         mbHasForcedPose{false};
    Sophus::SE3f mForcedPose;   // Tcw for the next frame

    // Saved initial-frame pose for the forced-initialization path (Phase 2).
    // Set when mInitialFrame is captured and a forced pose was provided.
    bool         mbInitialFrameHasForcedPose{false};
    Sophus::SE3f mInitialFrameForcedPose;

    // True when CreateInitialMapMonocular was called with both frames having
    // forced poses — map points are already in metric world coordinates so
    // the median-depth rescaling step must be skipped.
    bool         mbForcedPoseInitialization{false};

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization
    bool mFastInit = false;


    vector<MapPoint*> GetLocalMapMPS();

    bool mbWriteStats;

#ifdef REGISTER_TIMES
    void LocalMapStats2File();
    void TrackStats2File();
    void PrintTimeStats();

    vector<double> vdRectStereo_ms;
    vector<double> vdResizeImage_ms;
    vector<double> vdORBExtract_ms;
    vector<double> vdStereoMatch_ms;
    vector<double> vdIMUInteg_ms;
    vector<double> vdPosePred_ms;
    vector<double> vdLMTrack_ms;
    vector<double> vdNewKF_ms;
    vector<double> vdTrackTotal_ms;
#endif

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    //void CreateNewMapPoints();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    bool PredictStateIMU();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame(bool bForcedPose = false);

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();

    bool mbMapUpdated;

    // Imu preintegration from last frame
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //Feature extractor (ORB / AKAZE / SIFT, selected via Feature.type)
    FeatureExtractor* mpORBextractorLeft, *mpORBextractorRight;
    FeatureExtractor* mpIniORBextractor;
    FeatureType mFeatureType;

    // AKAZE detector parameters (read from optional AKAZE.* YAML keys).
    float mAkazeThreshold;
    int mAkazeNOctaves;
    int mAkazeNOctaveLayers;
    int mAkazeDescriptorSize;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    bool bStepByStep;

    //Atlas
    Atlas* mpAtlas;

    //Calibration matrix
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;
    float mImageScale;

    // --- Target-resolution dynamic downscaling (monocular service path) ------
    // When mTargetWidth > 0, incoming frames of any resolution are uniformly
    // scaled inside GrabImageMonocular so the binding side matches the target,
    // and the intrinsics are scaled by the same factor. The YAML / REST
    // intrinsics are always expressed at the camera's native resolution; the
    // scaled camera model is (re)built lazily once the live frame size is known.
    // Works with both config formats (legacy ParseCamParamFile and the
    // File.version 1.0 Settings/newParameterLoader path) — the keys are parsed
    // in the constructor and the native source intrinsics are captured lazily
    // from whichever camera model the loader built.
    int     mTargetWidth   = 0;   // internal processing width  (0 = disabled)
    int     mTargetHeight  = 0;   // internal processing height
    int     mCalibWidth    = 0;   // Camera.width  the intrinsics refer to
    int     mCalibHeight   = 0;   // Camera.height the intrinsics refer to
    float   mSrcFx = 0.f, mSrcFy = 0.f, mSrcCx = 0.f, mSrcCy = 0.f; // native K
    cv::Mat mSrcDistCoef;         // native distortion coefficients
    int     mSrcCameraType = 0;   // GeometricCamera::CAM_PINHOLE / CAM_FISHEYE
    bool    mbSrcCaptured  = false; // native source intrinsics captured
    bool    mbScaleDirty   = false; // rebuild scaled camera on next frame
    bool    mbManualCalib  = false; // REST calibration set → suppress res warning
    bool    mbResWarned    = false; // resolution-mismatch warning emitted once
    int     mLastFrameW    = 0, mLastFrameH = 0; // last incoming frame size

    // True when target-resolution downscaling is enabled (one or both target
    // dimensions configured).
    bool TargetModeActive() const { return mTargetWidth > 0 || mTargetHeight > 0; }
    // Parse the target-resolution YAML keys (camera-type / config-format
    // agnostic). Called once from the constructor.
    void ParseTargetResolution(cv::FileStorage &fSettings);
    // Capture the native source intrinsics from the active camera model the
    // loader built (lazy, on the first frame in target mode).
    void CaptureSourceIntrinsics();
    // Rebuild the active camera model from the native source intrinsics scaled
    // by s (used by the target-resolution path).
    void RebuildScaledCamera(float s);

    float mImuFreq;
    double mImuPer;
    bool mInsertKFsLost;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Minimum keypoint/match counts for monocular initialization (YAML: Tracking.minInitMatches)
    int mMinInitMatches;

    // Minimum camera baseline (metres) before ForcedPoseMonocularInit will triangulate (YAML: Tracking.forcedPoseMinBaseline)
    float mForcedPoseMinBaseline;

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    double mTimeStampLost;
    double time_recently_lost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;

    //Motion Model
    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    //int nMapChangeIndex;

    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera* mpCamera, *mpCamera2;

    int initID, lastID;

    Sophus::SE3f mTlr;

    void newParameterLoader(Settings* settings);

#ifdef REGISTER_LOOP
    bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif

public:
    cv::Mat mImRight;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
