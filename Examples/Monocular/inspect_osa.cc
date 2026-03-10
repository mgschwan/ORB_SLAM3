#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <boost/archive/binary_iarchive.hpp>

#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"

using namespace std;
using namespace ORB_SLAM3;

class MapWrapper : public Map {
public:
    std::vector<KeyFrame*> getBackupKeyFrames() {
        return mvpBackupKeyFrames;
    }
    std::vector<MapPoint*> getBackupMapPoints() {
        return mvpBackupMapPoints;
    }
};

class AtlasWrapper : public Atlas {
public:
    std::vector<Map*> getBackupMaps() {
        return mvpBackupMaps;
    }
};

int main(int argc, char **argv) {
    if(argc != 2) {
        cerr << "Usage: ./inspect_osa path_to_osa_file.osa" << endl;
        return 1;
    }

    string pathLoadFileName = argv[1];
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if(!ifs.good()) {
        cerr << "Error: File '" << pathLoadFileName << "' not found." << endl;
        return 1;
    }

    string strFileVoc, strVocChecksum;
    Atlas* mpAtlas = nullptr;

    cout << "Loading Atlas from: " << pathLoadFileName << " ..." << endl;
    
    try {
        boost::archive::binary_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
    } catch(std::exception &e) {
        cerr << "Exception during deserialization: " << e.what() << endl;
        return 1;
    }

    if(!mpAtlas) {
        cerr << "Failed to load Atlas object." << endl;
        return 1;
    }

    cout << "--------------------------------------------------------" << endl;
    cout << "Atlas Information" << endl;
    cout << "--------------------------------------------------------" << endl;
    cout << "Vocabulary file: " << strFileVoc << endl;
    cout << "Vocabulary checksum: " << strVocChecksum << endl;

    // During loading, maps are stored in mvpBackupMaps before PostLoad is called
    AtlasWrapper* pAtlasWrapper = static_cast<AtlasWrapper*>(mpAtlas);
    vector<Map*> maps = pAtlasWrapper->getBackupMaps();
    cout << "Total Maps: " << maps.size() << endl;

    for (size_t i = 0; i < maps.size(); ++i) {
        Map* pMap = maps[i];
        if(!pMap) continue;

        cout << endl;
        cout << "--- Map " << i << " (ID: " << pMap->GetId() << ") ---" << endl;
        
        // Similarly, KeyFrames and MapPoints are in backup vectors
        MapWrapper* pMapWrapper = static_cast<MapWrapper*>(pMap);
        vector<KeyFrame*> keyframes = pMapWrapper->getBackupKeyFrames();
        vector<MapPoint*> mappoints = pMapWrapper->getBackupMapPoints();

        cout << "  Total KeyFrames : " << keyframes.size() << endl;
        cout << "  Total MapPoints : " << mappoints.size() << endl;

        if (!keyframes.empty()) {
            double minTime = std::numeric_limits<double>::max();
            double maxTime = -1.0;
            long unsigned int minId = std::numeric_limits<long unsigned int>::max();
            long unsigned int maxId = 0;

            for (KeyFrame* pKF : keyframes) {
                if(!pKF) continue;
                
                if (pKF->mTimeStamp < minTime) minTime = pKF->mTimeStamp;
                if (pKF->mTimeStamp > maxTime) maxTime = pKF->mTimeStamp;
                
                if (pKF->mnId < minId) minId = pKF->mnId;
                if (pKF->mnId > maxId) maxId = pKF->mnId;
            }

            cout << "  KeyFrame Stats:" << endl;
            cout << "    - Min ID: " << minId << " | Max ID: " << maxId << endl;
            cout << "    - Min Timestamp: " << minTime << endl;
            cout << "    - Max Timestamp: " << maxTime << endl;
            cout << "    - Duration: " << (maxTime - minTime) << " s" << endl;
        }
    }

    cout << "--------------------------------------------------------" << endl;
    
    // We don't bother to delete mpAtlas properly since OS will clean up process memory
    return 0;
}
