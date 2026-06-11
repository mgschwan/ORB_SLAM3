#include "localization_service/preprocessor.h"

#include <sstream>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>

namespace localization_service {

namespace {

// ---------------------------------------------------------------------------
// CLAHE — Contrast Limited Adaptive Histogram Equalization.
// Color frames are converted to LAB and CLAHE is applied to the L (luminance)
// channel only, so colour is preserved while the contrast ORB-SLAM3 relies on
// is enhanced. Grayscale frames are equalized directly.
// ---------------------------------------------------------------------------
class ClaheStep : public PreprocessStep {
public:
    ClaheStep(double clipLimit, int tiles)
        : clahe_(cv::createCLAHE(clipLimit, cv::Size(tiles, tiles))),
          clipLimit_(clipLimit),
          tiles_(tiles)
    {}

    void apply(cv::Mat& frame) override
    {
        if (frame.empty())
            return;

        if (frame.channels() == 1) {
            clahe_->apply(frame, frame);
            return;
        }

        if (frame.channels() == 4)
            cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);

        cv::Mat lab;
        cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);

        std::vector<cv::Mat> ch;
        cv::split(lab, ch);
        clahe_->apply(ch[0], ch[0]);   // L channel only
        cv::merge(ch, lab);

        cv::cvtColor(lab, frame, cv::COLOR_Lab2BGR);
    }

    std::string describe() const override
    {
        std::ostringstream os;
        os << "CLAHE (clipLimit=" << clipLimit_
           << ", tiles=" << tiles_ << "x" << tiles_ << ")";
        return os.str();
    }

private:
    cv::Ptr<cv::CLAHE> clahe_;
    double             clipLimit_;
    int                tiles_;
};

// --- small typed FileStorage readers (tolerant of int/real nodes) -----------
int readInt(cv::FileStorage& fs, const char* key, int def)
{
    cv::FileNode n = fs[key];
    if (n.empty())   return def;
    if (n.isInt())   return static_cast<int>(n);
    if (n.isReal())  return static_cast<int>(n.real());
    return def;
}

double readDouble(cv::FileStorage& fs, const char* key, double def)
{
    cv::FileNode n = fs[key];
    if (n.empty())              return def;
    if (n.isReal() || n.isInt()) return static_cast<double>(n.real());
    return def;
}

} // namespace

FramePreprocessor FramePreprocessor::fromSettingsFile(const std::string& settingsPath)
{
    FramePreprocessor pp;

    cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
    if (!fs.isOpened())
        return pp;

    // --- CLAHE ---------------------------------------------------------------
    if (readInt(fs, "Preproc.clahe", 0) != 0) {
        const double clip  = readDouble(fs, "Preproc.claheClipLimit", 2.0);
        int          tiles = readInt(fs, "Preproc.claheTileSize", 8);
        if (tiles < 1) tiles = 1;
        pp.addStep(std::unique_ptr<PreprocessStep>(new ClaheStep(clip, tiles)));
    }

    // --- future steps: read their Preproc.* keys and append here, in the order
    //     they should run, e.g. denoise → gamma → sharpen.

    return pp;
}

void FramePreprocessor::addStep(std::unique_ptr<PreprocessStep> step)
{
    steps_.push_back(std::move(step));
}

void FramePreprocessor::process(cv::Mat& frame) const
{
    for (const auto& step : steps_)
        step->apply(frame);
}

std::string FramePreprocessor::describe() const
{
    if (steps_.empty())
        return "Preprocessing pipeline: (none)";

    std::ostringstream os;
    os << "Preprocessing pipeline:";
    for (const auto& step : steps_)
        os << "\n  - " << step->describe();
    return os.str();
}

} // namespace localization_service
