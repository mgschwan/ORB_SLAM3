#include "localization_service/slam_state.h"

namespace localization_service {

void PoseState::update(float x, float y, float z,
                       float qx, float qy, float qz, float qw) noexcept
{
    std::lock_guard<std::mutex> lock(mtx_);
    data_ = {x, y, z, qx, qy, qz, qw, true};
}

void PoseState::invalidate() noexcept
{
    std::lock_guard<std::mutex> lock(mtx_);
    data_.valid = false;
}

PoseState::Snapshot PoseState::snapshot() const noexcept
{
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
}

} // namespace localization_service
