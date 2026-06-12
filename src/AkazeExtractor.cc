/**
* This file is part of ORB-SLAM3 (modified).
*
* AkazeExtractor implementation. See AkazeExtractor.h for the scale-model notes.
*/

#include "AkazeExtractor.h"

#include <algorithm>
#include <numeric>

using namespace cv;
using namespace std;

namespace ORB_SLAM3
{

AkazeExtractor::AkazeExtractor(int nfeatures, float threshold, int nOctaves, int nOctaveLayers)
    : mnFeatures(nfeatures), mnLevels(nOctaves), mfScaleFactor(2.0f)
{
    if(mnLevels < 1)
        mnLevels = 1;
    if(nOctaveLayers < 1)
        nOctaveLayers = 1;

    // DESCRIPTOR_MLDB, full size (0 => 486 bits / 61 bytes), 3 channels.
    mpAkaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3,
                                threshold, mnLevels, nOctaveLayers);

    // Pyramid scale metadata: one level per octave, scale doubles each octave.
    mvScaleFactor.resize(mnLevels);
    mvLevelSigma2.resize(mnLevels);
    mvScaleFactor[0] = 1.0f;
    mvLevelSigma2[0] = 1.0f;
    for(int i = 1; i < mnLevels; i++)
    {
        mvScaleFactor[i] = mvScaleFactor[i-1] * mfScaleFactor;
        mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(mnLevels);
    mvInvLevelSigma2.resize(mnLevels);
    for(int i = 0; i < mnLevels; i++)
    {
        mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
        mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
    }
}

int AkazeExtractor::operator()(InputArray _image, InputArray _mask,
                               vector<KeyPoint>& _keypoints,
                               OutputArray _descriptors,
                               vector<int>& vLappingArea)
{
    if(_image.empty())
        return -1;

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1);

    vector<KeyPoint> kps;
    Mat desc;
    mpAkaze->detectAndCompute(image, _mask, kps, desc);

    if(kps.empty())
    {
        _keypoints.clear();
        _descriptors.release();
        return 0;
    }

    // Keep only the strongest mnFeatures keypoints to bound downstream cost.
    if(mnFeatures > 0 && (int)kps.size() > mnFeatures)
    {
        vector<int> idx(kps.size());
        iota(idx.begin(), idx.end(), 0);
        nth_element(idx.begin(), idx.begin() + mnFeatures, idx.end(),
                    [&kps](int a, int b){ return kps[a].response > kps[b].response; });
        idx.resize(mnFeatures);

        vector<KeyPoint> kps2;
        kps2.reserve(mnFeatures);
        Mat desc2(mnFeatures, desc.cols, desc.type());
        for(int i = 0; i < mnFeatures; i++)
        {
            kps2.push_back(kps[idx[i]]);
            desc.row(idx[i]).copyTo(desc2.row(i));
        }
        kps  = std::move(kps2);
        desc = desc2;
    }

    const int N = (int)kps.size();

    // Clamp octave to the valid pyramid range (AKAZE reports 0..nOctaves-1).
    for(int i = 0; i < N; i++)
    {
        int o = kps[i].octave;
        if(o < 0)            o = 0;
        if(o >= mnLevels)    o = mnLevels - 1;
        kps[i].octave = o;
    }

    // Split into mono (front) / stereo (back) by the lapping area, mirroring
    // ORBextractor::operator(). For monocular (vLappingArea == {0,0}) every
    // keypoint lands in the mono region and monoIndex == N.
    _keypoints = vector<KeyPoint>(N);
    _descriptors.create(N, desc.cols, CV_8U);
    Mat outDesc = _descriptors.getMat();

    int monoIndex = 0, stereoIndex = N - 1;
    for(int i = 0; i < N; i++)
    {
        const KeyPoint& kp = kps[i];
        if(kp.pt.x >= vLappingArea[0] && kp.pt.x <= vLappingArea[1])
        {
            _keypoints.at(stereoIndex) = kp;
            desc.row(i).copyTo(outDesc.row(stereoIndex));
            stereoIndex--;
        }
        else
        {
            _keypoints.at(monoIndex) = kp;
            desc.row(i).copyTo(outDesc.row(monoIndex));
            monoIndex++;
        }
    }

    return monoIndex;
}

} // namespace ORB_SLAM3
