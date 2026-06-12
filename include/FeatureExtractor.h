/**
* This file is part of ORB-SLAM3 (modified).
*
* FeatureExtractor: abstract interface for the feature detector + descriptor.
* ORBextractor is the reference implementation. The system is built around a
* single, uniform feature type per atlas (see Agents.md). The concrete type is
* selected at startup via the `Feature.type` YAML key and recorded in the
* serialized atlas so a map is always localized with the feature type it was
* built with.
*/

#ifndef FEATUREEXTRACTOR_H
#define FEATUREEXTRACTOR_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM3
{

// Feature/descriptor type. Stored (as int) in the serialized Atlas.
enum class FeatureType
{
    ORB   = 0,
    AKAZE = 1,
    SIFT  = 2
};

std::string FeatureTypeName(FeatureType t);
FeatureType FeatureTypeFromString(const std::string& s);

// Abstract feature extractor. The public surface mirrors what Frame consumes
// from the legacy ORBextractor (descriptor extraction + scale-pyramid metadata).
class FeatureExtractor
{
public:
    virtual ~FeatureExtractor() {}

    // Detect keypoints and compute descriptors on an image.
    virtual int operator()(cv::InputArray image, cv::InputArray mask,
                           std::vector<cv::KeyPoint>& keypoints,
                           cv::OutputArray descriptors,
                           std::vector<int>& vLappingArea) = 0;

    // Scale-pyramid metadata used by the matcher and optimizer.
    virtual int GetLevels() = 0;
    virtual float GetScaleFactor() = 0;
    virtual std::vector<float> GetScaleFactors() = 0;
    virtual std::vector<float> GetInverseScaleFactors() = 0;
    virtual std::vector<float> GetScaleSigmaSquares() = 0;
    virtual std::vector<float> GetInverseScaleSigmaSquares() = 0;

    virtual FeatureType GetType() const = 0;

    // Image pyramid, used only by stereo matching (Frame::ComputeStereoMatches).
    // Populated by ORBextractor; left empty by implementations that do not build
    // an explicit pyramid (monocular tracking does not use it).
    std::vector<cv::Mat> mvImagePyramid;
};

// Factory: build the extractor for the requested feature type. The first group
// of parameters follows the ORB extractor (nfeatures, scaleFactor, nlevels,
// iniThFAST, minThFAST). The trailing akaze* parameters configure the AKAZE
// scale space / detector and are ignored by other feature types (so existing
// ORB call sites are unaffected by the defaults).
FeatureExtractor* CreateFeatureExtractor(FeatureType type,
                                         int nfeatures, float scaleFactor,
                                         int nlevels, int iniThFAST, int minThFAST,
                                         float akazeThreshold = 0.001f,
                                         int akazeNOctaves = 4,
                                         int akazeNOctaveLayers = 4,
                                         int akazeDescriptorSize = 256);

} // namespace ORB_SLAM3

#endif // FEATUREEXTRACTOR_H
