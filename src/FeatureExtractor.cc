/**
* This file is part of ORB-SLAM3 (modified).
*
* Feature extractor factory and FeatureType helpers. Phase 1 implements ORB
* only; AKAZE and SIFT are added in later phases as additional branches of
* CreateFeatureExtractor.
*/

#include "FeatureExtractor.h"
#include "ORBextractor.h"
#include "AkazeExtractor.h"

#include <cctype>
#include <iostream>

namespace ORB_SLAM3
{

std::string FeatureTypeName(FeatureType t)
{
    switch(t)
    {
        case FeatureType::ORB:   return "ORB";
        case FeatureType::AKAZE: return "AKAZE";
        case FeatureType::SIFT:  return "SIFT";
    }
    return "ORB";
}

FeatureType FeatureTypeFromString(const std::string& s)
{
    std::string u;
    for(char c : s)
        u += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

    if(u == "AKAZE") return FeatureType::AKAZE;
    if(u == "SIFT")  return FeatureType::SIFT;
    return FeatureType::ORB;
}

FeatureExtractor* CreateFeatureExtractor(FeatureType type,
                                         int nfeatures, float scaleFactor,
                                         int nlevels, int iniThFAST, int minThFAST,
                                         float akazeThreshold,
                                         int akazeNOctaves,
                                         int akazeNOctaveLayers,
                                         int akazeDescriptorSize)
{
    switch(type)
    {
        case FeatureType::ORB:
            return new ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);

        case FeatureType::AKAZE:
            return new AkazeExtractor(nfeatures, akazeThreshold, akazeNOctaves,
                                      akazeNOctaveLayers, akazeDescriptorSize);

        case FeatureType::SIFT:
        default:
            std::cerr << "[FeatureExtractor] Feature type '" << FeatureTypeName(type)
                      << "' is not implemented yet; falling back to ORB." << std::endl;
            return new ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
    }
}

} // namespace ORB_SLAM3
