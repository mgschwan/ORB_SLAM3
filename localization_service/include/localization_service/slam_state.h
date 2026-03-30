#pragma once

#include <atomic>
#include <mutex>

namespace localization_service {

// ---- Lifecycle flags -------------------------------------------------------
// Plain struct: these are independent booleans with no invariant between them.
// std::atomic avoids a mutex for simple flag reads/writes across threads.
struct LifecycleFlags {
    std::atomic<bool> running{true};
    std::atomic<bool> paused{true};
};

// ---- Pose published by the tracking loop, consumed by the SSE thread -------
class PoseState {
public:
    struct Snapshot {
        float x{0}, y{0}, z{0};
        float qx{0}, qy{0}, qz{0}, qw{1};
        bool  valid{false};
    };

    // Called from the main tracking thread.
    void update(float x, float y, float z,
                float qx, float qy, float qz, float qw) noexcept;
    void invalidate() noexcept;

    // Returns a value-copy so callers never hold the mutex during formatting.
    Snapshot snapshot() const noexcept;

private:
    mutable std::mutex mtx_;
    Snapshot data_;
};

} // namespace localization_service
