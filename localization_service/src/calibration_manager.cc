#include "localization_service/calibration_manager.h"

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <System.h>

namespace localization_service {

CalibrationManager::CalibrationManager(ORB_SLAM3::System& slam)
    : slam_(slam)
{}

void CalibrationManager::enableMode(bool enable, std::atomic<bool>& paused)
{
    std::lock_guard<std::mutex> lock(mtx_);
    active_ = enable;
    if (enable)
        paused = true; // suspend SLAM tracking while calibrating
}

void CalibrationManager::requestCapture(float squareSize)
{
    std::lock_guard<std::mutex> lock(mtx_);
    captureRequested_ = true;
    if (squareSize >= 0.f)
        boardSquareSize_ = squareSize;
}

bool CalibrationManager::processFrame(const cv::Mat& frame)
{
    bool active, needCapture;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        active      = active_;
        needCapture = captureRequested_;
    }
    if (!active)
        return false;

    if (needCapture) {
        std::lock_guard<std::mutex> lock(mtx_);
        doCapture(frame);
        captureRequested_ = false;
    }
    return true;
}

// Called with mtx_ held.
void CalibrationManager::doCapture(const cv::Mat& frame)
{
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    const bool found = cv::findChessboardCorners(gray, boardSize_, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    cv::Mat display = frame.clone();
    cv::drawChessboardCorners(display, boardSize_, corners, found);
    latestImage_ = display;

    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        imagePoints_.push_back(corners);

        std::vector<cv::Point3f> obj;
        obj.reserve(boardSize_.width * boardSize_.height);
        for (int r = 0; r < boardSize_.height; ++r)
            for (int c = 0; c < boardSize_.width; ++c)
                obj.push_back(cv::Point3f(c * boardSquareSize_, r * boardSquareSize_, 0));
        objectPoints_.push_back(obj);

        ++captureCount_;
        lastCaptureSuccess_ = true;
    } else {
        lastCaptureSuccess_ = false;
    }
}

CalibrationManager::Status CalibrationManager::getStatus() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return {captureCount_, lastCaptureSuccess_, active_};
}

cv::Mat CalibrationManager::getLatestImage() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return latestImage_.empty() ? cv::Mat{} : latestImage_.clone();
}

std::string CalibrationManager::computeJson()
{
    std::vector<std::vector<cv::Point2f>> imgPts;
    std::vector<std::vector<cv::Point3f>> objPts;
    cv::Size imgSize;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        imgPts  = imagePoints_;
        objPts  = objectPoints_;
        if (!latestImage_.empty())
            imgSize = latestImage_.size();
    }

    if (imgPts.size() < 3 || imgSize.width == 0)
        return "{\"success\":false,\"error\":\"not enough captures\"}";

    cv::Mat K  = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D  = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    const double rms = cv::calibrateCamera(objPts, imgPts, imgSize, K, D, rvecs, tvecs);
    std::cout << "Calibration completed with RMS error: " << rms << std::endl;

    slam_.ChangeCalibration(K, D);

    {
        std::lock_guard<std::mutex> lock(mtx_);
        imagePoints_.clear();
        objectPoints_.clear();
        captureCount_ = 0;
        active_ = false;
    }

    return std::string("{\"success\":true")
        + ",\"fx\":"  + std::to_string(K.at<double>(0,0))
        + ",\"fy\":"  + std::to_string(K.at<double>(1,1))
        + ",\"cx\":"  + std::to_string(K.at<double>(0,2))
        + ",\"cy\":"  + std::to_string(K.at<double>(1,2))
        + ",\"k1\":"  + std::to_string(D.at<double>(0))
        + ",\"k2\":"  + std::to_string(D.at<double>(1))
        + ",\"p1\":"  + std::to_string(D.at<double>(2))
        + ",\"p2\":"  + std::to_string(D.at<double>(3))
        + ",\"k3\":"  + std::to_string(D.at<double>(4))
        + "}";
}

std::string CalibrationManager::applyManualJson(double fx, double fy,
                                                 double cx, double cy,
                                                 double k1, double k2,
                                                 double p1, double p2, double k3)
{
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0,0) = fx;
    K.at<double>(1,1) = fy;
    K.at<double>(0,2) = cx;
    K.at<double>(1,2) = cy;

    cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
    D.at<double>(0) = k1;
    D.at<double>(1) = k2;
    D.at<double>(2) = p1;
    D.at<double>(3) = p2;
    D.at<double>(4) = k3;

    slam_.ChangeCalibration(K, D);
    std::cout << ">>> [Web] Applied manual calibration from URL parameters <<<" << std::endl;
    return "{\"success\":true}";
}

} // namespace localization_service
