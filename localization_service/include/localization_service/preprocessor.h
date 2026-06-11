#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace localization_service {

// A single in-place image preprocessing operation (e.g. CLAHE, denoise, gamma).
// Steps run on the BGR/gray frame just before it is handed to ORB-SLAM3.
class PreprocessStep {
public:
    virtual ~PreprocessStep() = default;

    // Transform the frame in place.
    virtual void apply(cv::Mat& frame) = 0;

    // One-line human-readable description (parameters included) for startup logs.
    virtual std::string describe() const = 0;
};

// Ordered pipeline of preprocessing steps applied to every frame before tracking.
//
// To add a new step later:
//   1. Implement a PreprocessStep subclass in preprocessor.cc.
//   2. Read its `Preproc.*` keys and append it in fromSettingsFile(), in the
//      order it should run.
class FramePreprocessor {
public:
    // Build the pipeline from the `Preproc.*` keys of the YAML settings file.
    // Missing keys leave the corresponding step disabled. A settings file that
    // cannot be opened yields an empty (no-op) pipeline.
    static FramePreprocessor fromSettingsFile(const std::string& settingsPath);

    void addStep(std::unique_ptr<PreprocessStep> step);

    bool empty() const { return steps_.empty(); }

    // Run every step in order, modifying `frame` in place.
    void process(cv::Mat& frame) const;

    // Multi-line summary of the configured pipeline for startup logging.
    std::string describe() const;

private:
    std::vector<std::unique_ptr<PreprocessStep>> steps_;
};

} // namespace localization_service
