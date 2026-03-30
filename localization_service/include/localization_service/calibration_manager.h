#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM3 { class System; }

namespace localization_service {

// Manages chessboard-based camera calibration.
//
// Thread-safety: all public methods are safe to call from any thread.
// processFrame() is expected to be called from the main tracking thread only.
class CalibrationManager {
public:
    struct Status {
        int  count;
        bool lastSuccess;
        bool active;
    };

    struct CalibResult {
        bool   success{false};
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        double rms{0.0};
    };

    explicit CalibrationManager(ORB_SLAM3::System& slam);

    // Enable or disable calibration mode.
    // When enabling, also pass the paused flag so SLAM tracking is suspended.
    void enableMode(bool enable, std::atomic<bool>& paused);

    // Request a chessboard capture on the next processFrame() call.
    // squareSize overrides the default (0.025 m) when >= 0.
    void requestCapture(float squareSize = -1.f);

    // Called from the main tracking thread on each frame.
    // Returns true if calibration mode is active (caller should skip SLAM tracking).
    // When a capture has been requested, attempts chessboard detection and stores result.
    bool processFrame(const cv::Mat& frame);

    Status  getStatus()     const;
    cv::Mat getLatestImage() const; // returns a clone; empty if none yet

    // Runs cv::calibrateCamera on accumulated captures.
    // On success, calls SLAM.ChangeCalibration() and resets internal state.
    // Returns a JSON string ready to send as an HTTP response body.
    std::string computeJson();

    // Applies manually supplied intrinsics directly to SLAM.
    // Returns a JSON confirmation string.
    std::string applyManualJson(double fx, double fy, double cx, double cy,
                                double k1, double k2, double p1, double p2, double k3);

private:
    void doCapture(const cv::Mat& frame); // called with mtx_ held

    ORB_SLAM3::System& slam_;

    mutable std::mutex mtx_;
    bool   active_{false};
    bool   captureRequested_{false};
    float  boardSquareSize_{0.025f};
    cv::Size boardSize_{9, 6};

    std::vector<std::vector<cv::Point2f>> imagePoints_;
    std::vector<std::vector<cv::Point3f>> objectPoints_;
    cv::Mat latestImage_;
    int    captureCount_{0};
    bool   lastCaptureSuccess_{false};
};

} // namespace localization_service
