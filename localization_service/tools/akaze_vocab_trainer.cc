/**
* This file is part of ORB-SLAM3 (modified).
*
* akaze_vocab_trainer — trains a DBoW2 vocabulary from AKAZE descriptors.
*
* AKAZE is run at 256-bit M-LDB (32 bytes), which is byte-compatible with ORB,
* so the produced vocabulary is an ORBVocabulary (FORB / TemplatedVocabulary) in
* the exact same text format as ORBvoc.txt. At runtime, pass the resulting file
* as the vocabulary argument together with `Feature.type: "AKAZE"`.
*
* IMPORTANT: the AKAZE parameters used here (descriptor size, threshold,
* nOctaves, nOctaveLayers, nfeatures) must match the runtime YAML, otherwise the
* descriptor distribution the vocabulary was trained on will not match what the
* tracker produces.
*
* Usage:
*   akaze_vocab_trainer --out AKAZEvoc.txt [options] <imageDir> [<imageDir> ...]
*
* Options (defaults in brackets):
*   --out <file>         output vocabulary text file [AKAZEvoc.txt]
*   -k <int>             tree branching factor [10]
*   -L <int>             tree depth levels [6]   (k^L leaf words; 10^6 = ~1M)
*   --descriptor-size N  AKAZE M-LDB size in bits [256]  (256 => 32-byte / ORB-compatible)
*   --threshold <f>      AKAZE detector threshold [0.0008]
*   --noctaves <int>     AKAZE nOctaves [4]
*   --nlayers <int>      AKAZE nOctaveLayers [4]
*   --nfeatures <int>    max keypoints per image [1500]
*   --stride <int>       use every Nth image found (subsample video frames) [1]
*   --max-images <int>   stop after this many images (0 = no limit) [0]
*   --max-dim <int>      downscale so the longer image side <= this (0 = off) [0]
*                        (set to your runtime processing width, e.g. 640, to match scale)
*
* Image directories are scanned recursively for .jpg/.jpeg/.png/.bmp files.
*/

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "FeatureExtractor.h"
#include "ORBVocabulary.h"

using namespace std;
using namespace ORB_SLAM3;

struct Args {
    string out = "AKAZEvoc.txt";
    int k = 10;
    int L = 6;
    int descriptorSize = 256;
    float threshold = 0.0008f;
    int nOctaves = 4;
    int nOctaveLayers = 4;
    int nfeatures = 1500;
    int stride = 1;
    int maxImages = 0;
    int maxDim = 0;
    vector<string> dirs;
};

static bool hasImageExt(const string& path) {
    string ext;
    size_t dot = path.find_last_of('.');
    if (dot == string::npos) return false;
    for (size_t i = dot + 1; i < path.size(); ++i) ext += (char)tolower(path[i]);
    return ext == "jpg" || ext == "jpeg" || ext == "png" || ext == "bmp";
}

static bool parseArgs(int argc, char** argv, Args& a) {
    for (int i = 1; i < argc; ++i) {
        string s = argv[i];
        auto next = [&](const char* name) -> string {
            if (i + 1 >= argc) { cerr << "missing value for " << name << "\n"; exit(2); }
            return argv[++i];
        };
        if      (s == "--out")            a.out = next("--out");
        else if (s == "-k")               a.k = stoi(next("-k"));
        else if (s == "-L")               a.L = stoi(next("-L"));
        else if (s == "--descriptor-size")a.descriptorSize = stoi(next("--descriptor-size"));
        else if (s == "--threshold")      a.threshold = stof(next("--threshold"));
        else if (s == "--noctaves")       a.nOctaves = stoi(next("--noctaves"));
        else if (s == "--nlayers")        a.nOctaveLayers = stoi(next("--nlayers"));
        else if (s == "--nfeatures")      a.nfeatures = stoi(next("--nfeatures"));
        else if (s == "--stride")         a.stride = max(1, stoi(next("--stride")));
        else if (s == "--max-images")     a.maxImages = stoi(next("--max-images"));
        else if (s == "--max-dim")        a.maxDim = stoi(next("--max-dim"));
        else if (!s.empty() && s[0] == '-') { cerr << "unknown option: " << s << "\n"; return false; }
        else a.dirs.push_back(s);
    }
    if (a.dirs.empty()) {
        cerr << "Usage: akaze_vocab_trainer --out AKAZEvoc.txt [options] <imageDir> [<imageDir> ...]\n";
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    Args a;
    if (!parseArgs(argc, argv, a)) return 1;

    // Collect image files (recursive) from all input directories.
    vector<string> files;
    for (const string& dir : a.dirs) {
        vector<cv::String> found;
        try { cv::glob(dir + "/*", found, true); }
        catch (const cv::Exception& e) { cerr << "glob failed for " << dir << ": " << e.what() << "\n"; }
        for (const auto& f : found)
            if (hasImageExt(f)) files.push_back(f);
    }
    sort(files.begin(), files.end());

    // Subsample by stride, then cap by maxImages.
    vector<string> selected;
    for (size_t i = 0; i < files.size(); i += a.stride) {
        selected.push_back(files[i]);
        if (a.maxImages > 0 && (int)selected.size() >= a.maxImages) break;
    }

    cout << "Found " << files.size() << " images, using " << selected.size()
         << " (stride=" << a.stride << ", maxImages=" << a.maxImages << ")\n";
    cout << "AKAZE: descriptorSize=" << a.descriptorSize << " threshold=" << a.threshold
         << " nOctaves=" << a.nOctaves << " nOctaveLayers=" << a.nOctaveLayers
         << " nfeatures=" << a.nfeatures << "\n";
    cout << "Vocabulary: k=" << a.k << " L=" << a.L
         << " (=> up to " << (long long)pow((double)a.k, a.L) << " words)\n";
    if (selected.empty()) { cerr << "No images to train on.\n"; return 1; }

    FeatureExtractor* ex = CreateFeatureExtractor(
        FeatureType::AKAZE, a.nfeatures, 1.2f, 8, 20, 7,
        a.threshold, a.nOctaves, a.nOctaveLayers, a.descriptorSize);

    // Per-image descriptor sets for DBoW2 training.
    vector<vector<cv::Mat>> features;
    features.reserve(selected.size());

    auto t0 = chrono::steady_clock::now();
    size_t totalDesc = 0;
    int processed = 0, skipped = 0;
    for (const string& path : selected) {
        cv::Mat gray = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (gray.empty()) { skipped++; continue; }

        if (a.maxDim > 0) {
            int big = max(gray.cols, gray.rows);
            if (big > a.maxDim) {
                double s = (double)a.maxDim / big;
                cv::resize(gray, gray, cv::Size(), s, s, cv::INTER_AREA);
            }
        }

        vector<cv::KeyPoint> kps;
        cv::Mat desc;
        vector<int> vLap = {0, 0};
        (*ex)(gray, cv::Mat(), kps, desc, vLap);
        if (desc.empty()) { skipped++; continue; }

        vector<cv::Mat> rows;
        rows.reserve(desc.rows);
        for (int r = 0; r < desc.rows; ++r)
            rows.push_back(desc.row(r).clone());   // 1 x (size/8) CV_8U
        totalDesc += rows.size();
        features.push_back(std::move(rows));

        if (++processed % 200 == 0)
            cout << "  ...extracted " << processed << "/" << selected.size()
                 << " images, " << totalDesc << " descriptors\n" << flush;
    }
    delete ex;

    cout << "Extraction done: " << processed << " images ("
         << skipped << " skipped), " << totalDesc << " descriptors.\n";
    if (features.empty()) { cerr << "No descriptors extracted.\n"; return 1; }

    cout << "Training vocabulary (this can take a while)...\n" << flush;
    ORBVocabulary voc(a.k, a.L, DBoW2::TF_IDF, DBoW2::L1_NORM);
    voc.create(features);
    cout << "Vocabulary created: " << voc << "\n";

    cout << "Saving to " << a.out << " ...\n" << flush;
    voc.saveToTextFile(a.out);

    auto secs = chrono::duration_cast<chrono::seconds>(
                    chrono::steady_clock::now() - t0).count();
    cout << "Done in " << secs << "s. Wrote " << a.out << "\n";
    return 0;
}
