/**
* This file is part of ORB-SLAM3 (modified).
*
* AkazeExtractor: FeatureExtractor implementation backed by cv::AKAZE.
*
* AKAZE produces binary M-LDB descriptors compared with Hamming distance,
* exactly like ORB, so the existing ORBmatcher::DescriptorDistance (generalized
* to arbitrary length) and the matching pipeline are reused unchanged.
*
* Descriptor width: by default we request a 256-bit (32-byte) M-LDB descriptor
* (descriptorSize=256). At 32 bytes it is byte-compatible with ORB, so the DBoW2
* FORB descriptor class and the whole ORBVocabulary / Bag-of-Words plumbing are
* reused unchanged -- an AKAZE vocabulary is simply a differently trained
* vocabulary file of the same type (see tools/akaze_vocab_trainer.cc). Passing
* descriptorSize=0 selects AKAZE's full 486-bit (61-byte) descriptor instead.
*
* Scale model: cv::AKAZE reports keypoint.octave in [0, nOctaves) with the
* feature scale (keypoint.size) doubling each octave. We therefore expose the
* extractor as a pyramid of nOctaves levels with a scale factor of 2.0; the
* keypoint.octave field is already a valid level index, so no remapping of the
* nonlinear scale space is needed. Sub-octave layers (nOctaveLayers) increase
* detection density but are collapsed onto their octave for the SLAM scale model.
*
* NOTE: relocalization / loop closing use a Bag-of-Words vocabulary which is
* descriptor-type specific. Until an AKAZE vocabulary is trained (Phase 3), BoW
* runs against the ORB vocabulary and is only weakly meaningful; frame-to-frame
* tracking and mapping work because they match descriptors directly.
*/

#ifndef AKAZEEXTRACTOR_H
#define AKAZEEXTRACTOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "FeatureExtractor.h"

namespace ORB_SLAM3
{

class AkazeExtractor : public FeatureExtractor
{
public:
    // nfeatures: cap on the number of (strongest) keypoints kept; <=0 means keep all.
    // threshold: AKAZE detector response threshold.
    // nOctaves / nOctaveLayers: AKAZE scale-space structure (nOctaves == #levels).
    // descriptorSize: M-LDB descriptor size in bits (256 => 32 bytes, ORB-compatible;
    //                 0 => full 486-bit / 61-byte descriptor).
    AkazeExtractor(int nfeatures, float threshold, int nOctaves, int nOctaveLayers,
                   int descriptorSize = 256);

    ~AkazeExtractor() {}

    int operator()(cv::InputArray image, cv::InputArray mask,
                   std::vector<cv::KeyPoint>& keypoints,
                   cv::OutputArray descriptors,
                   std::vector<int>& vLappingArea) override;

    int GetLevels() override { return mnLevels; }
    float GetScaleFactor() override { return mfScaleFactor; }
    std::vector<float> GetScaleFactors() override { return mvScaleFactor; }
    std::vector<float> GetInverseScaleFactors() override { return mvInvScaleFactor; }
    std::vector<float> GetScaleSigmaSquares() override { return mvLevelSigma2; }
    std::vector<float> GetInverseScaleSigmaSquares() override { return mvInvLevelSigma2; }

    FeatureType GetType() const override { return FeatureType::AKAZE; }

private:
    cv::Ptr<cv::AKAZE> mpAkaze;

    int   mnFeatures;     // keypoint cap (<=0: unlimited)
    int   mnLevels;       // == nOctaves
    float mfScaleFactor;  // == 2.0 (AKAZE octaves double the scale)

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} // namespace ORB_SLAM3

#endif // AKAZEEXTRACTOR_H
