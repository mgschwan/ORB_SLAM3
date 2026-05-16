#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>

#include <opencv2/core/mat.hpp>
#include <sophus/se3.hpp>

namespace localization_service {

struct IngestFrame {
    cv::Mat image;
    double  timestamp{0.0};  // milliseconds (same unit as TrackMonocular tframe)

    // Optional IMU measurement attached to this frame.
    bool    hasImu{false};
    float   ax{0}, ay{0}, az{0};  // accelerometer  m/s²
    float   gx{0}, gy{0}, gz{0};  // gyroscope      rad/s

    // Optional external pose for this frame (Tcw: transforms world → camera,
    // ORB-SLAM3 internal convention).  When set, Tracking skips its feature-
    // based pose estimator and pose optimizer and uses this pose directly.
    bool         hasPose{false};
    Sophus::SE3f Tcw;
};

// Thread-safe, bounded frame queue shared between the ingest route(s) and the
// main tracking loop.  Bounded to kMaxDepth so a slow tracker never builds up
// a backlog of stale frames.
//
// Option 1: POST /api/frame   → push() one frame per request
// Option 2: multipart stream  → push() in a reader thread (future)
class IngestQueue {
public:
    static constexpr int kMaxDepth = 2;

    // Push a frame. Returns false (frame dropped) if the queue is already full.
    bool push(IngestFrame frame);

    // Block until a frame is available or timeoutUs elapses.
    // Returns false on timeout.
    bool pop(IngestFrame& out, int timeoutUs);

private:
    std::mutex              mtx_;
    std::condition_variable cv_;
    std::deque<IngestFrame> q_;
};

} // namespace localization_service
