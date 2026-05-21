/**
 * ForcedPoseInit.cc — Monocular initialization using externally provided camera poses.
 *
 * When both the initial frame and the current frame carry forced poses from an
 * external source (e.g. Unity), we can skip TwoViewReconstruction's H/F estimator
 * entirely.  Instead, we:
 *   1. Gate on a minimum baseline to ensure acceptable depth accuracy.
 *   2. Filter matches with the epipolar constraint derived from the known T21.
 *   3. Triangulate surviving matches with the known projection matrices.
 *   4. Apply parallax, depth-range, and reprojection-error filters.
 *   5. Convert the resulting camera-1-frame points to world coordinates using
 *      the initial frame's forced pose.
 *   6. Skip GlobalBundleAdjustment (it would shift the world reference frame).
 *
 * Part of ORB-SLAM3 localization service extensions.
 */

#include "Tracking.h"

#include "GeometricTools.h"
#include "Frame.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <cmath>

using namespace std;

namespace ORB_SLAM3 {

// ---------------------------------------------------------------------------
// Thresholds — tweak here rather than hunting through MonocularInitialization
// ---------------------------------------------------------------------------

static constexpr float kMinBaseline      = 0.15f;   // metres: wait for this much camera travel
static constexpr float kEpipolarTh       = 2.5f;    // Sampson distance in pixels
static constexpr float kReprojTh2        = 4.0f;    // reprojection error threshold (pixels²)
static constexpr float kMinCosParallax   = 0.9998f; // ≈1.1° min parallax
static constexpr float kMinDepth         = 0.05f;   // metres — below this is an artefact
static constexpr float kMaxDepth         = 50.0f;   // metres
static constexpr int   kMinGoodPoints    = 30;       // minimum to accept initialization

// ---------------------------------------------------------------------------
// Tracking::ForcedPoseMonocularInit
//
// Called from MonocularInitialization when both mbInitialFrameHasForcedPose
// and mbHasForcedPose are set.  On success, calls CreateInitialMapMonocular().
// In all cases (success, wait, or failure), the caller should return immediately.
// ---------------------------------------------------------------------------
void Tracking::ForcedPoseMonocularInit(int nmatches)
{
    // ---- Compute T21 (initial → current camera transform) ----------------
    Sophus::SE3f T21 = mForcedPose * mInitialFrameForcedPose.inverse();
    Eigen::Matrix3f R = T21.rotationMatrix();
    Eigen::Vector3f t = T21.translation();

    // ---- Diagnostic -------------------------------------------------------
    {
        Eigen::AngleAxisf aa(R);
        Eigen::Vector3f twc_init = mInitialFrameForcedPose.inverse().translation();
        Eigen::Vector3f twc_cur  = mForcedPose.inverse().translation();
        cout << "[ForcedPoseInit] init cam (Twc.t): " << twc_init.transpose() << endl;
        cout << "[ForcedPoseInit] cur  cam (Twc.t): " << twc_cur.transpose() << endl;
        cout << "[ForcedPoseInit] T21 t=" << t.transpose()
             << "  |t|=" << t.norm() << "m"
             << "  rot=" << aa.axis().transpose()
             << " " << aa.angle() * 180.0f / float(M_PI) << "deg" << endl;
    }

    // ---- Baseline gate ----------------------------------------------------
    if(t.norm() < kMinBaseline)
    {
        cout << "[ForcedPoseInit] baseline=" << t.norm()
             << "m < " << kMinBaseline << "m, waiting" << endl;
        return;
    }

    // ---- Camera intrinsics and projection matrices -------------------------
    const float fx = Frame::fx, fy = Frame::fy;
    const float cx = Frame::cx, cy = Frame::cy;
    Eigen::Matrix3f K;
    K << fx,  0, cx,
          0, fy, cy,
          0,  0,  1;

    Eigen::Matrix<float,3,4> P1;
    P1.setZero();
    P1.block<3,3>(0,0) = K;

    Eigen::Matrix<float,3,4> P2;
    P2.block<3,3>(0,0) = R;
    P2.block<3,1>(0,3) = t;
    P2 = K * P2;

    // ---- Fundamental matrix from known T21 --------------------------------
    // E = [t]_x * R,  F = K^{-T} * E * K^{-1}
    Eigen::Matrix3f tx;
    tx <<      0, -t(2),  t(1),
           t(2),      0, -t(0),
          -t(1),  t(0),      0;
    Eigen::Matrix3f Kinv = K.inverse();
    Eigen::Matrix3f F21  = Kinv.transpose() * (tx * R) * Kinv;

    // Camera centres in camera-1 frame
    Eigen::Vector3f O1(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f O2 = -(R.transpose() * t);

    // ---- Triangulation loop -----------------------------------------------
    mvIniP3D.resize(mInitialFrame.mvKeysUn.size());
    vector<bool> vbTriangulated(mInitialFrame.mvKeysUn.size(), false);

    int nGood        = 0;
    int nEpipolar    = 0;
    int nNonFinite   = 0;
    int nBehind      = 0;
    int nLowParallax = 0;
    int nDepthRange  = 0;
    int nReproj      = 0;

    for(size_t i = 0; i < mvIniMatches.size(); i++)
    {
        if(mvIniMatches[i] < 0)
            continue;

        const cv::KeyPoint& kp1 = mInitialFrame.mvKeysUn[i];
        const cv::KeyPoint& kp2 = mCurrentFrame.mvKeysUn[mvIniMatches[i]];

        Eigen::Vector3f x1(kp1.pt.x, kp1.pt.y, 1.0f);
        Eigen::Vector3f x2(kp2.pt.x, kp2.pt.y, 1.0f);

        // Epipolar filter (Sampson distance): rejects mismatches before
        // triangulation so we don't waste time on outliers.
        {
            Eigen::Vector3f Fx1  = F21 * x1;
            Eigen::Vector3f Ftx2 = F21.transpose() * x2;
            float x2tFx1  = x2.dot(Fx1);
            float sampson  = x2tFx1 * x2tFx1 /
                             (Fx1(0)*Fx1(0)   + Fx1(1)*Fx1(1) +
                              Ftx2(0)*Ftx2(0) + Ftx2(1)*Ftx2(1));
            if(std::abs(sampson) > kEpipolarTh) { nEpipolar++; continue; }
        }

        Eigen::Vector3f p3dC1;
        GeometricTools::Triangulate(x1, x2, P1, P2, p3dC1);

        if(!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2)))
            { nNonFinite++; continue; }

        if(p3dC1(2) <= 0.0f)
            { nBehind++; continue; }

        Eigen::Vector3f p3dC2 = R * p3dC1 + t;
        if(p3dC2(2) <= 0.0f)
            { nBehind++; continue; }

        // Parallax: low-parallax points have unreliable depths and scatter
        // to large distances, polluting the map.
        Eigen::Vector3f n1 = p3dC1 - O1;
        Eigen::Vector3f n2 = p3dC1 - O2;
        float cosParallax = n1.dot(n2) / (n1.norm() * n2.norm());
        if(cosParallax >= kMinCosParallax) { nLowParallax++; continue; }

        // Depth range: remove extremely close or far artefacts.
        if(p3dC1(2) < kMinDepth || p3dC1(2) > kMaxDepth)
            { nDepthRange++; continue; }

        // Reprojection error in camera 1
        float invZ1 = 1.0f / p3dC1(2);
        float err1 = (fx*p3dC1(0)*invZ1 + cx - kp1.pt.x) * (fx*p3dC1(0)*invZ1 + cx - kp1.pt.x)
                   + (fy*p3dC1(1)*invZ1 + cy - kp1.pt.y) * (fy*p3dC1(1)*invZ1 + cy - kp1.pt.y);
        if(err1 > kReprojTh2) { nReproj++; continue; }

        // Reprojection error in camera 2
        float invZ2 = 1.0f / p3dC2(2);
        float err2 = (fx*p3dC2(0)*invZ2 + cx - kp2.pt.x) * (fx*p3dC2(0)*invZ2 + cx - kp2.pt.x)
                   + (fy*p3dC2(1)*invZ2 + cy - kp2.pt.y) * (fy*p3dC2(1)*invZ2 + cy - kp2.pt.y);
        if(err2 > kReprojTh2) { nReproj++; continue; }

        mvIniP3D[i] = cv::Point3f(p3dC1(0), p3dC1(1), p3dC1(2));
        vbTriangulated[i] = true;
        nGood++;
    }

    cout << "[ForcedPoseInit] matches=" << nmatches
         << " good=" << nGood
         << " | epipolar=" << nEpipolar
         << " behind=" << nBehind
         << " nonfinite=" << nNonFinite
         << " depthRange=" << nDepthRange
         << " lowParallax=" << nLowParallax
         << " reproj=" << nReproj << endl;

    // ---- Debug visualization ----------------------------------------------
    // Saves /tmp/orbslam_init_matches.png with green (good) / red (failed) matches.
    if(!mInitImGray.empty() && !mImGray.empty())
    {
        cv::Mat canvas;
        cv::hconcat(mInitImGray, mImGray, canvas);
        if(canvas.channels() == 1)
            cv::cvtColor(canvas, canvas, cv::COLOR_GRAY2BGR);

        const int xOffset = mInitImGray.cols;
        for(size_t i = 0; i < mvIniMatches.size(); i++)
        {
            if(mvIniMatches[i] < 0) continue;
            const cv::KeyPoint& kp1 = mInitialFrame.mvKeysUn[i];
            const cv::KeyPoint& kp2 = mCurrentFrame.mvKeysUn[mvIniMatches[i]];
            cv::Point2f p1(kp1.pt.x, kp1.pt.y);
            cv::Point2f p2(kp2.pt.x + xOffset, kp2.pt.y);
            bool good = vbTriangulated[i];
            cv::Scalar dot  = good ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
            cv::Scalar line = good ? cv::Scalar(0,180,0) : cv::Scalar(0,0,180);
            cv::circle(canvas, p1, 3, dot,  -1);
            cv::circle(canvas, p2, 3, dot,  -1);
            cv::line(canvas, p1, p2, line, 1);
        }
        cv::imwrite("/tmp/orbslam_init_matches.png", canvas);
        cout << "[ForcedPoseInit] viz: /tmp/orbslam_init_matches.png"
             << " (green=good, red=failed)" << endl;
    }

    if(nGood < kMinGoodPoints)
    {
        cout << "[ForcedPoseInit] only " << nGood << " good points (need "
             << kMinGoodPoints << "), waiting" << endl;
        return;
    }

    // ---- Strip untrianglated matches and count survivors ------------------
    int nFinal = 0;
    for(size_t i = 0; i < mvIniMatches.size(); i++)
    {
        if(mvIniMatches[i] >= 0 && !vbTriangulated[i])
            mvIniMatches[i] = -1;
        else if(mvIniMatches[i] >= 0)
            nFinal++;
    }

    // ---- Transform cam-1 points to world coordinates ----------------------
    Sophus::SE3f Twc_init = mInitialFrameForcedPose.inverse();
    for(size_t i = 0; i < mvIniP3D.size(); i++)
    {
        if(!vbTriangulated[i]) continue;
        Eigen::Vector3f p_cam(mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z);
        Eigen::Vector3f p_world = Twc_init * p_cam;
        mvIniP3D[i] = cv::Point3f(p_world.x(), p_world.y(), p_world.z());
    }

    // ---- Lock in the forced poses -----------------------------------------
    mInitialFrame.SetPose(mInitialFrameForcedPose);
    mCurrentFrame.SetPose(mForcedPose);
    mbHasForcedPose          = false;
    mbForcedPoseInitialization = true;

    cout << "[ForcedPoseInit] SUCCESS — triangulated " << nFinal << " points, creating map" << endl;
    CreateInitialMapMonocular();
}

} // namespace ORB_SLAM3
