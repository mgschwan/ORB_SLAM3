/**
 * osa_convert.cc — Dump an OSA atlas to JSON and pack JSON back to OSA.
 *
 * Usage:
 *   osa_convert dump <input.osa>  <output.json>
 *   osa_convert pack <input.json> <output.osa>
 *
 * Binary blobs (cv::Mat data) are base64-encoded strings in the JSON.
 * SE3 poses are stored as {qw,qx,qy,qz,tx,ty,tz}.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>

#include <nlohmann/json.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Frame.h"
#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

using namespace std;
using namespace ORB_SLAM3;
using json = nlohmann::json;

// ─── Base64 ──────────────────────────────────────────────────────────────────

static const string B64_CHARS =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static string base64_encode(const unsigned char* data, size_t len) {
    string out;
    out.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3) {
        unsigned char b0 = data[i];
        unsigned char b1 = (i+1 < len) ? data[i+1] : 0;
        unsigned char b2 = (i+2 < len) ? data[i+2] : 0;
        out += B64_CHARS[(b0 >> 2) & 0x3F];
        out += B64_CHARS[((b0 & 0x03) << 4) | ((b1 >> 4) & 0x0F)];
        out += (i+1 < len) ? B64_CHARS[((b1 & 0x0F) << 2) | ((b2 >> 6) & 0x03)] : '=';
        out += (i+2 < len) ? B64_CHARS[b2 & 0x3F] : '=';
    }
    return out;
}

static vector<uint8_t> base64_decode(const string& enc) {
    static const int T[256] = {
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,62,-1,-1,-1,63,
        52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-1,-1,-1,
        -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,
        15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,-1,-1,
        -1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
        41,42,43,44,45,46,47,48,49,50,51,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1
    };
    vector<uint8_t> out;
    out.reserve(enc.size() * 3 / 4);
    int val = 0, bits = -8;
    for (unsigned char c : enc) {
        if (T[c] == -1) continue;
        val = (val << 6) + T[c];
        bits += 6;
        if (bits >= 0) { out.push_back((val >> bits) & 0xFF); bits -= 8; }
    }
    return out;
}

// ─── cv::Mat ↔ JSON ──────────────────────────────────────────────────────────

static json mat_to_json(const cv::Mat& mat) {
    cv::Mat m = mat;
    if (!m.isContinuous()) m = m.clone();
    size_t bytes = (size_t)m.rows * m.cols * m.elemSize();
    return {{"cols",m.cols},{"rows",m.rows},{"type",m.type()},
            {"data", base64_encode(m.data, bytes)}};
}

static cv::Mat json_to_mat(const json& j) {
    int cols = j["cols"], rows = j["rows"], type = j["type"];
    cv::Mat m(rows, cols, type);
    auto bytes = base64_decode(j["data"].get<string>());
    size_t expected = (size_t)rows * cols * m.elemSize();
    if (bytes.size() >= expected) memcpy(m.data, bytes.data(), expected);
    return m;
}

// ─── SE3 ↔ JSON ──────────────────────────────────────────────────────────────

static json se3_to_json(const Sophus::SE3f& T) {
    Eigen::Quaternionf q = T.unit_quaternion();
    Eigen::Vector3f t = T.translation();
    return {{"qw",q.w()},{"qx",q.x()},{"qy",q.y()},{"qz",q.z()},
            {"tx",t.x()},{"ty",t.y()},{"tz",t.z()}};
}

static Sophus::SE3f json_to_se3(const json& j) {
    Eigen::Quaternionf q((float)j["qw"],(float)j["qx"],(float)j["qy"],(float)j["qz"]);
    Eigen::Vector3f t((float)j["tx"],(float)j["ty"],(float)j["tz"]);
    return Sophus::SE3f(q, t);
}

// ─── KeyPoint ↔ JSON ─────────────────────────────────────────────────────────

static json kp_to_json(const cv::KeyPoint& kp) {
    return {{"angle",kp.angle},{"response",kp.response},{"size",kp.size},
            {"x",kp.pt.x},{"y",kp.pt.y},{"class_id",kp.class_id},{"octave",kp.octave}};
}

static cv::KeyPoint json_to_kp(const json& j) {
    cv::KeyPoint kp;
    kp.angle=(float)j["angle"]; kp.response=(float)j["response"]; kp.size=(float)j["size"];
    kp.pt.x=(float)j["x"]; kp.pt.y=(float)j["y"];
    kp.class_id=j["class_id"]; kp.octave=j["octave"];
    return kp;
}

// ─── IMU bias ↔ JSON ─────────────────────────────────────────────────────────

static json bias_to_json(const IMU::Bias& b) {
    auto s = [](float f){ return (std::isnan(f)||std::isinf(f)) ? 0.0f : f; };
    return {{"bax",s(b.bax)},{"bay",s(b.bay)},{"baz",s(b.baz)},
            {"bwx",s(b.bwx)},{"bwy",s(b.bwy)},{"bwz",s(b.bwz)}};
}

static IMU::Bias json_to_bias(const json& j) {
    return IMU::Bias((float)j["bax"],(float)j["bay"],(float)j["baz"],
                     (float)j["bwx"],(float)j["bwy"],(float)j["bwz"]);
}

// ─── IMU Calib ↔ JSON ────────────────────────────────────────────────────────

static json imu_calib_to_json(const IMU::Calib& c) {
    IMU::Calib& cc = const_cast<IMU::Calib&>(c);
    json j;
    j["tcb"] = se3_to_json(cc.mTcb);
    j["tbc"] = se3_to_json(cc.mTbc);
    j["cov"]      = vector<float>(cc.Cov.diagonal().data(),     cc.Cov.diagonal().data()+6);
    j["cov_walk"] = vector<float>(cc.CovWalk.diagonal().data(), cc.CovWalk.diagonal().data()+6);
    j["is_set"]   = cc.mbIsSet;
    return j;
}

static void fill_imu_calib(const json& j, IMU::Calib& c) {
    c.mTcb    = json_to_se3(j["tcb"]);
    c.mTbc    = json_to_se3(j["tbc"]);
    vector<float> cov  = j["cov"];
    vector<float> covw = j["cov_walk"];
    for (int i = 0; i < 6; i++) {
        c.Cov.diagonal()[i]     = cov[i];
        c.CovWalk.diagonal()[i] = covw[i];
    }
    c.mbIsSet = j["is_set"];
}

// ─── NaN-safe float helper ────────────────────────────────────────────────────

static json safe_floats(const float* data, size_t n) {
    json arr = json::array();
    for (size_t i = 0; i < n; i++) {
        float v = data[i];
        arr.push_back((std::isnan(v) || std::isinf(v)) ? 0.0f : v);
    }
    return arr;
}

// ─── IMU Preintegrated ↔ JSON (non-copyable → fill existing object) ───────────

static json imu_preint_to_json(const IMU::Preintegrated& p) {
    IMU::Preintegrated& mp = const_cast<IMU::Preintegrated&>(p);
    json j;
    float dT = mp.dT; j["dT"] = (std::isnan(dT)||std::isinf(dT)) ? 0.0f : dT;
    j["C"]      = safe_floats(mp.C.data(), 225);
    j["Info"]   = safe_floats(mp.Info.data(), 225);
    j["Nga"]    = safe_floats(mp.Nga.diagonal().data(),    6);
    j["NgaWalk"]= safe_floats(mp.NgaWalk.diagonal().data(),6);
    j["b"]      = bias_to_json(mp.b);
    j["dR"]     = safe_floats(mp.dR.data(), 9);
    j["dV"]     = safe_floats(mp.dV.data(), 3);
    j["dP"]     = safe_floats(mp.dP.data(), 3);
    j["JRg"]    = safe_floats(mp.JRg.data(), 9);
    j["JVg"]    = safe_floats(mp.JVg.data(), 9);
    j["JVa"]    = safe_floats(mp.JVa.data(), 9);
    j["JPg"]    = safe_floats(mp.JPg.data(), 9);
    j["JPa"]    = safe_floats(mp.JPa.data(), 9);
    j["avgA"]   = safe_floats(mp.avgA.data(), 3);
    j["avgW"]   = safe_floats(mp.avgW.data(), 3);
    j["bu"]     = bias_to_json(mp.GetUpdatedBias());
    Eigen::Matrix<float,6,1> db = mp.GetDeltaBias();
    j["db"]     = safe_floats(db.data(), 6);
    j["measurements"] = json::array(); // private — empty ok for non-inertial
    return j;
}

static void fill_imu_preint(const json& j, IMU::Preintegrated& p) {
    p.dT = j["dT"];
    vector<float> C = j["C"], Info = j["Info"];
    memcpy(p.C.data(),    C.data(),    225*sizeof(float));
    memcpy(p.Info.data(), Info.data(), 225*sizeof(float));
    vector<float> Nga = j["Nga"], NgaWalk = j["NgaWalk"];
    for (int i=0;i<6;i++) { p.Nga.diagonal()[i]=Nga[i]; p.NgaWalk.diagonal()[i]=NgaWalk[i]; }
    p.b = json_to_bias(j["b"]);
    vector<float> dR=j["dR"],dV=j["dV"],dP=j["dP"];
    vector<float> JRg=j["JRg"],JVg=j["JVg"],JVa=j["JVa"],JPg=j["JPg"],JPa=j["JPa"];
    vector<float> avgA=j["avgA"],avgW=j["avgW"];
    memcpy(p.dR.data(),dR.data(),9*sizeof(float));
    memcpy(p.dV.data(),dV.data(),3*sizeof(float));
    memcpy(p.dP.data(),dP.data(),3*sizeof(float));
    memcpy(p.JRg.data(),JRg.data(),9*sizeof(float));
    memcpy(p.JVg.data(),JVg.data(),9*sizeof(float));
    memcpy(p.JVa.data(),JVa.data(),9*sizeof(float));
    memcpy(p.JPg.data(),JPg.data(),9*sizeof(float));
    memcpy(p.JPa.data(),JPa.data(),9*sizeof(float));
    memcpy(p.avgA.data(),avgA.data(),3*sizeof(float));
    memcpy(p.avgW.data(),avgW.data(),3*sizeof(float));
}

// ─── Wrapper classes for protected member access ──────────────────────────────

struct MapWrapper : public Map {
    // DUMP accessors
    const vector<KeyFrame*>& backupKFs() const { return mvpBackupKeyFrames; }
    const vector<MapPoint*>& backupMPs() const { return mvpBackupMapPoints; }
    const vector<unsigned long int>& originsIds() const { return mvBackupKeyFrameOriginsId; }
    unsigned long initKFid()       const { return mnInitKFid; }
    unsigned long maxKFid()        const { return mnMaxKFid; }
    int           bigChangeIdx()   const { return mnBigChangeIdx; }
    bool          imuInitialized() const { return mbImuInitialized; }
    bool          isInertial()     const { return mbIsInertial; }
    bool          imuBA1()         const { return mbIMU_BA1; }
    bool          imuBA2()         const { return mbIMU_BA2; }

    // PACK setters
    void setId(unsigned long v)           { mnId = v; }
    void setInitKFid(unsigned long v)     { mnInitKFid = v; }
    void setMaxKFid(unsigned long v)      { mnMaxKFid = v; }
    void setBigChangeIdx(int v)           { mnBigChangeIdx = v; }
    void setImuInitialized(bool v)        { mbImuInitialized = v; }
    void setIsInertial(bool v)            { mbIsInertial = v; }
    void setImuBA1(bool v)                { mbIMU_BA1 = v; }
    void setImuBA2(bool v)                { mbIMU_BA2 = v; }
    void setBackupKFinitialID(unsigned long v) { mnBackupKFinitialID = v; }
    void setBackupKFlowerID(unsigned long v)   { mnBackupKFlowerID = v; }
    void addBackupKF(KeyFrame* kf)        { mvpBackupKeyFrames.push_back(kf); }
    void addBackupMP(MapPoint* mp)        { mvpBackupMapPoints.push_back(mp); }
    void addOriginId(unsigned long v)     { mvBackupKeyFrameOriginsId.push_back(v); }
};

struct AtlasWrapper : public Atlas {
    const vector<Map*>& backupMaps()              const { return mvpBackupMaps; }
    const vector<GeometricCamera*>& cameras()     const { return mvpCameras; }
    unsigned long lastInitKFidMap()               const { return mnLastInitKFidMap; }
    void setLastInitKFidMap(unsigned long v)      { mnLastInitKFidMap = v; }
    void addBackupMap(Map* m)                     { mvpBackupMaps.push_back(m); }
    void addCamera(GeometricCamera* c)            { mvpCameras.push_back(c); }
};

struct KeyFrameAccess : public KeyFrame {
    // ─ DUMP getters for protected/not-directly-accessible fields ─
    bool  getFirstConn()    const { return mbFirstConnection; }
    bool  getNotErase()     const { return mbNotErase; }
    bool  getToBeErased()   const { return mbToBeErased; }
    bool  getBad()          const { return mbBad; }
    float getHalfBaseline() const { return mHalfBaseline; }
    bool  getHasVelocity()  const { return mbHasVelocity; }
    const Sophus::SE3f& getTlr()  const { return mTlr; }
    const cv::Mat& getDistCoef()  const { return mDistCoef; }
    const Eigen::Matrix3f& getK() const { return mK_; }
    const DBoW2::BowVector&    getBowVec()  const { return mBowVec; }
    const DBoW2::FeatureVector& getFeatVec() const { return mFeatVec; }
    const vector<long long int>& getMapPointIds() const { return mvBackupMapPointsId; }
    const vector<vector<vector<size_t>>>& getGrid()      const { return mGrid; }
    const vector<vector<vector<size_t>>>& getGridRight() const { return mGridRight; }
    const map<long unsigned int,int>& getConnKFWeights() const { return mBackupConnectedKeyFrameIdWeights; }
    long long getBackupParentId() const { return mBackupParentId; }
    const vector<unsigned long>& getChildrenIds()  const { return mvBackupChildrensId; }
    const vector<unsigned long>& getLoopEdgeIds()  const { return mvBackupLoopEdgesId; }
    const vector<unsigned long>& getMergeEdgeIds() const { return mvBackupMergeEdgesId; }
    unsigned int getCameraId()  const { return mnBackupIdCamera; }
    unsigned int getCamera2Id() const { return mnBackupIdCamera2; }
    const IMU::Bias&          getImuBias()          const { return mImuBias; }
    const IMU::Preintegrated& getBackupImuPreint()  const { return mBackupImuPreintegrated; }
    const IMU::Calib&         getImuCalib()         const { return mImuCalib; }
    long long getPrevKFId()  const { return mBackupPrevKFId; }
    long long getNextKFId()  const { return mBackupNextKFId; }
    const Eigen::Vector3f& getVw()  const { return mVw; }
    const Eigen::Vector3f& getOwb() const { return mOwb; }

    // ─ PACK setters for const fields ─
    void setId(unsigned long v)           { mnId = v; }
    void setFrameId(unsigned long v)      { const_cast<long unsigned int&>(mnFrameId) = v; }
    void setTimestamp(double v)           { const_cast<double&>(mTimeStamp) = v; }
    void setGridCols(int v)               { const_cast<int&>(mnGridCols) = v; }
    void setGridRows(int v)               { const_cast<int&>(mnGridRows) = v; }
    void setGridWidthInv(float v)         { const_cast<float&>(mfGridElementWidthInv) = v; }
    void setGridHeightInv(float v)        { const_cast<float&>(mfGridElementHeightInv) = v; }
    void setFx(float v)                   { const_cast<float&>(fx) = v; }
    void setFy(float v)                   { const_cast<float&>(fy) = v; }
    void setInvFx(float v)                { const_cast<float&>(invfx) = v; }
    void setInvFy(float v)                { const_cast<float&>(invfy) = v; }
    void setCx(float v)                   { const_cast<float&>(cx) = v; }
    void setCy(float v)                   { const_cast<float&>(cy) = v; }
    void setMbf(float v)                  { const_cast<float&>(mbf) = v; }
    void setMb(float v)                   { const_cast<float&>(mb) = v; }
    void setThDepth(float v)              { const_cast<float&>(mThDepth) = v; }
    void setN(int v)                      { const_cast<int&>(N) = v; }
    void setKeys(const vector<cv::KeyPoint>& v)    { const_cast<vector<cv::KeyPoint>&>(mvKeys) = v; }
    void setKeysUn(const vector<cv::KeyPoint>& v)  { const_cast<vector<cv::KeyPoint>&>(mvKeysUn) = v; }
    void setURight(const vector<float>& v)         { const_cast<vector<float>&>(mvuRight) = v; }
    void setDepth(const vector<float>& v)          { const_cast<vector<float>&>(mvDepth) = v; }
    void setDescriptors(const cv::Mat& v)          { const_cast<cv::Mat&>(mDescriptors) = v; }
    void setScaleLevels(int v)            { const_cast<int&>(mnScaleLevels) = v; }
    void setScaleFactor(float v)          { const_cast<float&>(mfScaleFactor) = v; }
    void setLogScaleFactor(float v)       { const_cast<float&>(mfLogScaleFactor) = v; }
    void setScaleFactors(const vector<float>& v)   { const_cast<vector<float>&>(mvScaleFactors) = v; }
    void setLevelSigma2(const vector<float>& v)    { const_cast<vector<float>&>(mvLevelSigma2) = v; }
    void setInvLevelSigma2(const vector<float>& v) { const_cast<vector<float>&>(mvInvLevelSigma2) = v; }
    void setMinX(int v) { const_cast<int&>(mnMinX) = v; }
    void setMinY(int v) { const_cast<int&>(mnMinY) = v; }
    void setMaxX(int v) { const_cast<int&>(mnMaxX) = v; }
    void setMaxY(int v) { const_cast<int&>(mnMaxY) = v; }
    void setNLeft(int v)  { const_cast<int&>(NLeft) = v; }
    void setNRight(int v) { const_cast<int&>(NRight) = v; }
    void setKeysRight(const vector<cv::KeyPoint>& v) { const_cast<vector<cv::KeyPoint>&>(mvKeysRight) = v; }
    // ─ PACK setters for protected fields ─
    void setK(const Eigen::Matrix3f& v)   { mK_ = v; }
    void setTcw(const Sophus::SE3f& v)    { mTcw = v; }
    void setTcp(const Sophus::SE3f& v)    { mTcp = v; }
    void setTlr(const Sophus::SE3f& v)    { mTlr = v; }
    void setDistCoef(const cv::Mat& v)    { mDistCoef = v; }
    void setBowVec(const DBoW2::BowVector& v)      { mBowVec = v; }
    void setFeatVec(const DBoW2::FeatureVector& v) { mFeatVec = v; }
    void setMapPointIds(const vector<long long int>& v) { mvBackupMapPointsId = v; }
    void setGrid(const vector<vector<vector<size_t>>>& v)      { mGrid = v; }
    void setGridRight(const vector<vector<vector<size_t>>>& v) { mGridRight = v; }
    void setConnKFWeights(const map<long unsigned int,int>& v) { mBackupConnectedKeyFrameIdWeights = v; }
    void setFirstConnection(bool v)       { mbFirstConnection = v; }
    void setBackupParentId(long long v)   { mBackupParentId = v; }
    void setChildrenIds(const vector<unsigned long>& v) { mvBackupChildrensId = v; }
    void setLoopEdgeIds(const vector<unsigned long>& v) { mvBackupLoopEdgesId = v; }
    void setMergeEdgeIds(const vector<unsigned long>& v){ mvBackupMergeEdgesId = v; }
    void setNotErase(bool v)              { mbNotErase = v; }
    void setToBeErased(bool v)            { mbToBeErased = v; }
    void setBad(bool v)                   { mbBad = v; }
    void setHalfBaseline(float v)         { mHalfBaseline = v; }
    void setOriginMapId(unsigned int v)   { mnOriginMapId = v; }
    void setCameraId(unsigned int v)      { mnBackupIdCamera = v; }
    void setCamera2Id(unsigned int v)     { mnBackupIdCamera2 = v; }
    void setLeftToRightMatch(const vector<int>& v)  { mvLeftToRightMatch = v; }
    void setRightToLeftMatch(const vector<int>& v)  { mvRightToLeftMatch = v; }
    void setImuBias(const IMU::Bias& v)   { mImuBias = v; }
    void fillBackupImuPreint(const json& j) { fill_imu_preint(j, mBackupImuPreintegrated); }
    void fillImuCalib(const json& j)       { fill_imu_calib(j, mImuCalib); }
    void setPrevKFId(long long v)         { mBackupPrevKFId = v; }
    void setNextKFId(long long v)         { mBackupNextKFId = v; }
    void setBImu(bool v)                  { bImu = v; }
    void setVw(const Eigen::Vector3f& v)  { mVw = v; }
    void setOwb(const Eigen::Vector3f& v) { mOwb = v; }
    void setHasVelocity(bool v)           { mbHasVelocity = v; }
    void setScale(float v)                { mfScale = v; }
};

struct MapPointAccess : public MapPoint {
    // ─ DUMP getters ─
    bool  getBad()      const { return mbBad; }
    float getMinDist()  const { return mfMinDistance; }
    float getMaxDist()  const { return mfMaxDistance; }
    const Eigen::Vector3f& worldPos()  const { return mWorldPos; }
    const Eigen::Vector3f& normal()    const { return mNormalVector; }
    const map<unsigned long,int>& obs1() const { return mBackupObservationsId1; }
    const map<unsigned long,int>& obs2() const { return mBackupObservationsId2; }
    const cv::Mat& descriptor()        const { return mDescriptor; }
    unsigned long refKFId()            const { return mBackupRefKFId; }
    long long replacedId()             const { return mBackupReplacedId; }
    // ─ PACK setters ─
    void setId(unsigned long v)           { mnId = v; }
    void setWorldPos(const Eigen::Vector3f& v) { mWorldPos = v; }
    void setNormal(const Eigen::Vector3f& v)   { mNormalVector = v; }
    void setObs1(const map<unsigned long,int>& v) { mBackupObservationsId1 = v; }
    void setObs2(const map<unsigned long,int>& v) { mBackupObservationsId2 = v; }
    void setDescriptor(const cv::Mat& v)  { mDescriptor = v; }
    void setRefKFId(unsigned long v)      { mBackupRefKFId = v; }
    void setBad(bool v)                   { mbBad = v; }
    void setReplacedId(long long v)       { mBackupReplacedId = v; }
    void setMinDist(float v)              { mfMinDistance = v; }
    void setMaxDist(float v)              { mfMaxDistance = v; }
};

// ─── Grid ↔ JSON ─────────────────────────────────────────────────────────────

static json grid_to_json(const vector<vector<vector<size_t>>>& g) {
    json jg = json::array();
    for (const auto& col : g) {
        json jcol = json::array();
        for (const auto& row : col) {
            json jrow = json::array();
            for (size_t v : row) jrow.push_back(v);
            jcol.push_back(jrow);
        }
        jg.push_back(jcol);
    }
    return jg;
}

static vector<vector<vector<size_t>>> json_to_grid(const json& j) {
    vector<vector<vector<size_t>>> g;
    for (const auto& jcol : j) {
        vector<vector<size_t>> col;
        for (const auto& jrow : jcol) {
            vector<size_t> row;
            for (const auto& v : jrow) row.push_back(v.get<size_t>());
            col.push_back(row);
        }
        g.push_back(col);
    }
    return g;
}

// ─── DUMP ─────────────────────────────────────────────────────────────────────

static json mappoint_to_json(MapPoint* mp_) {
    auto* mp = static_cast<MapPointAccess*>(mp_);
    json j;
    j["id"]          = mp->mnId;
    j["first_kf_id"] = mp->mnFirstKFid;
    j["first_frame"] = mp->mnFirstFrame;
    j["n_obs"]       = mp->nObs;
    j["world_pos"]   = {mp->worldPos().x(), mp->worldPos().y(), mp->worldPos().z()};
    j["normal"]      = {mp->normal().x(), mp->normal().y(), mp->normal().z()};

    json obs1, obs2;
    for (auto it = mp->obs1().begin(); it != mp->obs1().end(); ++it)
        obs1[to_string(it->first)] = it->second;
    for (auto it = mp->obs2().begin(); it != mp->obs2().end(); ++it)
        obs2[to_string(it->first)] = it->second;
    j["obs1"] = obs1;
    j["obs2"] = obs2;

    j["descriptor"]  = mat_to_json(mp->descriptor());
    j["ref_kf_id"]   = mp->refKFId();
    j["bad"]         = mp->getBad();
    j["replaced_id"] = mp->replacedId();
    j["min_dist"]    = mp->getMinDist();
    j["max_dist"]    = mp->getMaxDist();
    return j;
}

static json keyframe_to_json(KeyFrame* kf_) {
    auto* kf = static_cast<KeyFrameAccess*>(kf_);
    json j;
    j["id"]              = kf->mnId;
    j["frame_id"]        = kf->mnFrameId;
    j["timestamp"]       = kf->mTimeStamp;
    j["grid_cols"]       = kf->mnGridCols;
    j["grid_rows"]       = kf->mnGridRows;
    j["grid_width_inv"]  = kf->mfGridElementWidthInv;
    j["grid_height_inv"] = kf->mfGridElementHeightInv;
    {
        float s = kf->mfScale;
        j["scale"] = (std::isnan(s) || std::isinf(s)) ? 1.0f : s;
    }
    j["fx"]=kf->fx; j["fy"]=kf->fy; j["invfx"]=kf->invfx; j["invfy"]=kf->invfy;
    j["cx"]=kf->cx; j["cy"]=kf->cy; j["mbf"]=kf->mbf; j["mb"]=kf->mb;
    j["th_depth"]        = kf->mThDepth;
    j["dist_coef"]       = mat_to_json(kf->getDistCoef());
    j["n"]               = kf->N;

    json kps = json::array(), kpsun = json::array();
    for (const auto& k : kf->mvKeys)   kps.push_back(kp_to_json(k));
    for (const auto& k : kf->mvKeysUn) kpsun.push_back(kp_to_json(k));
    j["keypoints"]    = kps;
    j["keypoints_un"] = kpsun;
    j["u_right"]      = kf->mvuRight;
    j["depth"]        = kf->mvDepth;
    j["descriptors"]  = mat_to_json(kf->mDescriptors);

    json bv, fv;
    for (auto it = kf->getBowVec().begin(); it != kf->getBowVec().end(); ++it)
        bv[to_string(it->first)] = it->second;
    for (auto it = kf->getFeatVec().begin(); it != kf->getFeatVec().end(); ++it) {
        json arr = json::array();
        for (unsigned int idx : it->second) arr.push_back(idx);
        fv[to_string(it->first)] = arr;
    }
    j["bow_vec"]  = bv;
    j["feat_vec"] = fv;

    j["tcp"]             = se3_to_json(kf->mTcp);
    j["scale_levels"]    = kf->mnScaleLevels;
    j["scale_factor"]    = kf->mfScaleFactor;
    j["log_scale_factor"]= kf->mfLogScaleFactor;
    j["scale_factors"]   = kf->mvScaleFactors;
    j["level_sigma2"]    = kf->mvLevelSigma2;
    j["inv_level_sigma2"]= kf->mvInvLevelSigma2;
    j["min_x"]=kf->mnMinX; j["min_y"]=kf->mnMinY;
    j["max_x"]=kf->mnMaxX; j["max_y"]=kf->mnMaxY;

    vector<float> K(kf->getK().data(), kf->getK().data()+9);
    j["K"]               = K;
    j["tcw"]             = se3_to_json(kf->GetPose());
    j["mappoint_ids"]    = kf->getMapPointIds();
    j["grid"]            = grid_to_json(kf->getGrid());
    j["grid_right"]      = grid_to_json(kf->getGridRight());

    json connw;
    for (auto it = kf->getConnKFWeights().begin(); it != kf->getConnKFWeights().end(); ++it)
        connw[to_string(it->first)] = it->second;
    j["connected_kf_weights"] = connw;

    j["first_connection"] = kf->getFirstConn();
    j["parent_id"]        = kf->getBackupParentId();
    j["children_ids"]     = kf->getChildrenIds();
    j["loop_edge_ids"]    = kf->getLoopEdgeIds();
    j["merge_edge_ids"]   = kf->getMergeEdgeIds();
    j["not_erase"]        = kf->getNotErase();
    j["to_be_erased"]     = kf->getToBeErased();
    j["bad"]              = kf->getBad();
    j["half_baseline"]    = kf->getHalfBaseline();
    j["origin_map_id"]    = kf->mnOriginMapId;
    j["camera_id"]        = kf->getCameraId();
    j["camera2_id"]       = kf->getCamera2Id();
    j["left_to_right"]    = kf->mvLeftToRightMatch;
    j["right_to_left"]    = kf->mvRightToLeftMatch;
    j["n_left"]           = kf->NLeft;
    j["n_right"]          = kf->NRight;
    j["tlr"]              = se3_to_json(kf->getTlr());

    json kpsr = json::array();
    for (const auto& k : kf->mvKeysRight) kpsr.push_back(kp_to_json(k));
    j["keypoints_right"]  = kpsr;

    j["imu_bias"]         = bias_to_json(kf->getImuBias());
    j["imu_preint"]       = imu_preint_to_json(kf->getBackupImuPreint());
    j["imu_calib"]        = imu_calib_to_json(kf->getImuCalib());
    j["prev_kf_id"]       = kf->getPrevKFId();
    j["next_kf_id"]       = kf->getNextKFId();
    j["b_imu"]            = kf->bImu;
    auto safeV3 = [](const Eigen::Vector3f& v) -> json {
        auto s = [](float f){ return (std::isnan(f)||std::isinf(f)) ? 0.0f : f; };
        return json::array({s(v.x()), s(v.y()), s(v.z())});
    };
    j["vw"]               = safeV3(kf->getVw());
    j["owb"]              = safeV3(kf->getOwb());
    j["has_velocity"]     = kf->getHasVelocity();
    return j;
}

static json map_to_json(Map* m_) {
    auto* m = static_cast<MapWrapper*>(m_);
    json j;
    j["id"]             = m->GetId();
    j["init_kf_id"]     = m->initKFid();
    j["max_kf_id"]      = m->maxKFid();
    j["big_change_idx"] = m->bigChangeIdx();
    j["imu_initialized"]= m->imuInitialized();
    j["is_inertial"]    = m->isInertial();
    j["imu_ba1"]        = m->imuBA1();
    j["imu_ba2"]        = m->imuBA2();

    json kfs = json::array();
    for (KeyFrame* kf : m->backupKFs()) if (kf) kfs.push_back(keyframe_to_json(kf));
    json mps = json::array();
    for (MapPoint* mp : m->backupMPs()) if (mp) mps.push_back(mappoint_to_json(mp));
    j["keyframes"] = kfs;
    j["mappoints"] = mps;

    json origids = json::array();
    for (unsigned long v : m->originsIds()) origids.push_back(v);
    j["kf_origins_ids"] = origids;
    return j;
}

static int cmd_dump(const string& osa_path, const string& json_path) {
    ifstream ifs(osa_path, ios::binary);
    if (!ifs.good()) { cerr << "Cannot open: " << osa_path << "\n"; return 1; }

    string vocab_name, vocab_checksum;
    Atlas* pAtlas = nullptr;
    try {
        boost::archive::binary_iarchive ia(ifs);
        ia >> vocab_name >> vocab_checksum >> pAtlas;
    } catch (exception& e) {
        cerr << "Deserialization error: " << e.what() << "\n"; return 1;
    }

    auto* atlas = static_cast<AtlasWrapper*>(pAtlas);
    json root;
    root["vocab_name"]     = vocab_name;
    root["vocab_checksum"] = vocab_checksum;
    root["map_next_id"]    = Map::nNextId;
    root["frame_next_id"]  = Frame::nNextId;
    root["kf_next_id"]     = KeyFrame::nNextId;
    root["mp_next_id"]     = MapPoint::nNextId;
    root["cam_next_id"]    = GeometricCamera::nNextId;
    root["last_init_kfid"] = atlas->lastInitKFidMap();

    json cameras = json::array();
    for (GeometricCamera* cam : atlas->cameras()) {
        json jc;
        jc["id"] = cam->GetId(); jc["type"] = cam->GetType();
        vector<float> params;
        for (size_t i = 0; i < cam->size(); i++) params.push_back(cam->getParameter(i));
        jc["params"] = params;
        cameras.push_back(jc);
    }
    root["cameras"] = cameras;

    json maps = json::array();
    for (Map* m : atlas->backupMaps()) if (m) maps.push_back(map_to_json(m));
    root["maps"] = maps;

    ofstream ofs(json_path);
    ofs << root.dump(2);
    ofs.close();
    cout << "Dumped " << atlas->backupMaps().size() << " map(s) to " << json_path << "\n";
    delete pAtlas;
    return 0;
}

// ─── PACK ─────────────────────────────────────────────────────────────────────

static MapPoint* json_to_mappoint(const json& j) {
    auto* mp = new MapPointAccess();
    mp->setId(j["id"]);
    mp->mnFirstKFid  = j["first_kf_id"];
    mp->mnFirstFrame = j["first_frame"];
    mp->nObs         = j["n_obs"];
    vector<float> wp = j["world_pos"];
    mp->setWorldPos(Eigen::Vector3f(wp[0], wp[1], wp[2]));
    vector<float> nv = j["normal"];
    mp->setNormal(Eigen::Vector3f(nv[0], nv[1], nv[2]));

    map<unsigned long,int> obs1, obs2;
    for (auto it = j["obs1"].begin(); it != j["obs1"].end(); ++it)
        obs1[stoul(it.key())] = it.value();
    for (auto it = j["obs2"].begin(); it != j["obs2"].end(); ++it)
        obs2[stoul(it.key())] = it.value();
    mp->setObs1(obs1);
    mp->setObs2(obs2);

    mp->setDescriptor(json_to_mat(j["descriptor"]));
    mp->setRefKFId(j["ref_kf_id"]);
    mp->setBad(j["bad"]);
    mp->setReplacedId(j["replaced_id"]);
    mp->setMinDist(j["min_dist"]);
    mp->setMaxDist(j["max_dist"]);
    return mp;
}

static KeyFrame* json_to_keyframe(const json& j) {
    auto* kf = new KeyFrameAccess();
    kf->setId(j["id"]);
    kf->setFrameId(j["frame_id"]);
    kf->setTimestamp(j["timestamp"]);
    kf->setGridCols(j["grid_cols"]);
    kf->setGridRows(j["grid_rows"]);
    kf->setGridWidthInv(j["grid_width_inv"]);
    kf->setGridHeightInv(j["grid_height_inv"]);
    // scale set below after has_velocity (may be null/NaN)
    kf->setFx(j["fx"]);    kf->setFy(j["fy"]);
    kf->setInvFx(j["invfx"]); kf->setInvFy(j["invfy"]);
    kf->setCx(j["cx"]);    kf->setCy(j["cy"]);
    kf->setMbf(j["mbf"]);  kf->setMb(j["mb"]);
    kf->setThDepth(j["th_depth"]);
    kf->setDistCoef(json_to_mat(j["dist_coef"]));
    kf->setN(j["n"]);

    vector<cv::KeyPoint> kps, kpsun;
    for (const auto& jk : j["keypoints"])    kps.push_back(json_to_kp(jk));
    for (const auto& jk : j["keypoints_un"]) kpsun.push_back(json_to_kp(jk));
    kf->setKeys(kps);
    kf->setKeysUn(kpsun);
    kf->setURight(j["u_right"].get<vector<float>>());
    kf->setDepth(j["depth"].get<vector<float>>());
    kf->setDescriptors(json_to_mat(j["descriptors"]));

    DBoW2::BowVector bv;
    for (auto it = j["bow_vec"].begin(); it != j["bow_vec"].end(); ++it)
        bv[stoul(it.key())] = it.value().get<double>();
    kf->setBowVec(bv);

    DBoW2::FeatureVector fv;
    for (auto it = j["feat_vec"].begin(); it != j["feat_vec"].end(); ++it) {
        vector<unsigned int> idxs;
        for (const auto& vi : it.value()) idxs.push_back(vi.get<unsigned int>());
        fv[stoul(it.key())] = idxs;
    }
    kf->setFeatVec(fv);

    kf->setTcp(json_to_se3(j["tcp"]));
    kf->setScaleLevels(j["scale_levels"]);
    kf->setScaleFactor(j["scale_factor"]);
    kf->setLogScaleFactor(j["log_scale_factor"]);
    kf->setScaleFactors(j["scale_factors"].get<vector<float>>());
    kf->setLevelSigma2(j["level_sigma2"].get<vector<float>>());
    kf->setInvLevelSigma2(j["inv_level_sigma2"].get<vector<float>>());
    kf->setMinX(j["min_x"]); kf->setMinY(j["min_y"]);
    kf->setMaxX(j["max_x"]); kf->setMaxY(j["max_y"]);

    vector<float> K = j["K"];
    Eigen::Matrix3f Km;
    memcpy(Km.data(), K.data(), 9*sizeof(float));
    kf->setK(Km);
    kf->setTcw(json_to_se3(j["tcw"]));

    kf->setMapPointIds(j["mappoint_ids"].get<vector<long long int>>());
    kf->setGrid(json_to_grid(j["grid"]));
    kf->setGridRight(json_to_grid(j["grid_right"]));

    map<long unsigned int,int> connw;
    for (auto it = j["connected_kf_weights"].begin(); it != j["connected_kf_weights"].end(); ++it)
        connw[stoul(it.key())] = it.value();
    kf->setConnKFWeights(connw);

    kf->setFirstConnection(j["first_connection"]);
    kf->setBackupParentId(j["parent_id"]);
    kf->setChildrenIds(j["children_ids"].get<vector<unsigned long>>());
    kf->setLoopEdgeIds(j["loop_edge_ids"].get<vector<unsigned long>>());
    kf->setMergeEdgeIds(j["merge_edge_ids"].get<vector<unsigned long>>());
    kf->setNotErase(j["not_erase"]);
    kf->setToBeErased(j["to_be_erased"]);
    kf->setBad(j["bad"]);
    kf->setHalfBaseline(j["half_baseline"]);
    kf->setOriginMapId(j["origin_map_id"]);
    kf->setCameraId(j["camera_id"]);
    kf->setCamera2Id(j["camera2_id"]);
    kf->setLeftToRightMatch(j["left_to_right"].get<vector<int>>());
    kf->setRightToLeftMatch(j["right_to_left"].get<vector<int>>());
    kf->setNLeft(j["n_left"]);
    kf->setNRight(j["n_right"]);
    vector<cv::KeyPoint> kpr;
    for (const auto& jk : j["keypoints_right"]) kpr.push_back(json_to_kp(jk));
    kf->setKeysRight(kpr);
    kf->setTlr(json_to_se3(j["tlr"]));

    kf->setImuBias(json_to_bias(j["imu_bias"]));
    kf->fillBackupImuPreint(j["imu_preint"]);
    kf->fillImuCalib(j["imu_calib"]);
    kf->setPrevKFId(j["prev_kf_id"]);
    kf->setNextKFId(j["next_kf_id"]);
    kf->setBImu(j["b_imu"]);
    vector<float> vw = j["vw"], owb = j["owb"];
    kf->setVw(Eigen::Vector3f(vw[0], vw[1], vw[2]));
    kf->setOwb(Eigen::Vector3f(owb[0], owb[1], owb[2]));
    kf->setHasVelocity(j["has_velocity"]);
    {
        auto& js = j["scale"];
        kf->setScale(js.is_null() ? 1.0f : js.get<float>());
    }
    return kf;
}

static int cmd_pack(const string& json_path, const string& osa_path) {
    ifstream jf(json_path);
    if (!jf.good()) { cerr << "Cannot open: " << json_path << "\n"; return 1; }
    json root = json::parse(jf);

    // Restore static IDs before creating any objects
    Map::nNextId             = root["map_next_id"];
    Frame::nNextId           = root["frame_next_id"];
    KeyFrame::nNextId        = root["kf_next_id"];
    MapPoint::nNextId        = root["mp_next_id"];
    GeometricCamera::nNextId = root["cam_next_id"];

    // Build cameras — constructors auto-assign mnId via nNextId++ so reset it first
    GeometricCamera::nNextId = 0;
    vector<GeometricCamera*> cameras;
    for (const auto& jc : root["cameras"]) {
        unsigned int type = jc["type"];
        vector<float> params = jc["params"];
        GeometricCamera* cam;
        if (type == GeometricCamera::CAM_PINHOLE)
            cam = new Pinhole(params);
        else
            cam = new KannalaBrandt8(params);
        cameras.push_back(cam);
    }
    // Restore the next-id counter after camera construction
    GeometricCamera::nNextId = root["cam_next_id"];

    auto* atlas = new AtlasWrapper();
    atlas->setLastInitKFidMap(root["last_init_kfid"]);
    for (auto* c : cameras) atlas->addCamera(c);

    for (const auto& jm : root["maps"]) {
        auto* m = new MapWrapper();
        m->setId(jm["id"]);
        m->setInitKFid(jm["init_kf_id"]);
        m->setMaxKFid(jm["max_kf_id"]);
        m->setBigChangeIdx(jm["big_change_idx"]);
        m->setImuInitialized(jm["imu_initialized"]);
        m->setIsInertial(jm["is_inertial"]);
        m->setImuBA1(jm["imu_ba1"]);
        m->setImuBA2(jm["imu_ba2"]);
        for (const auto& v : jm["kf_origins_ids"]) m->addOriginId(v.get<unsigned long>());

        for (const auto& jkf : jm["keyframes"]) m->addBackupKF(json_to_keyframe(jkf));
        for (const auto& jmp : jm["mappoints"])  m->addBackupMP(json_to_mappoint(jmp));

        // Derive backup initial/lower KF IDs
        if (!jm["keyframes"].empty()) {
            unsigned long firstId = jm["keyframes"][0]["id"];
            unsigned long lowerId = firstId;
            for (const auto& jkf : jm["keyframes"]) {
                unsigned long kid = jkf["id"];
                if (kid < lowerId) lowerId = kid;
            }
            m->setBackupKFinitialID(firstId);
            m->setBackupKFlowerID(lowerId);
        }
        atlas->addBackupMap(m);
    }

    string outPath = osa_path;
    if (outPath.size() < 4 || outPath.substr(outPath.size()-4) != ".osa")
        outPath += ".osa";
    remove(outPath.c_str());
    ofstream ofs(outPath, ios::binary);
    if (!ofs.good()) { cerr << "Cannot write: " << outPath << "\n"; return 1; }

    string vocab_name     = root["vocab_name"];
    string vocab_checksum = root["vocab_checksum"];
    try {
        boost::archive::binary_oarchive oa(ofs);
        oa << vocab_name << vocab_checksum << static_cast<Atlas*>(atlas);
    } catch (exception& e) {
        cerr << "Serialization error: " << e.what() << "\n"; return 1;
    }
    cout << "Packed " << root["maps"].size() << " map(s) to " << outPath << "\n";
    return 0;
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    if (argc < 4) {
        cerr << "Usage:\n"
             << "  osa_convert dump <input.osa>  <output.json>\n"
             << "  osa_convert pack <input.json> <output.osa>\n";
        return 1;
    }
    string mode = argv[1];
    if (mode == "dump") return cmd_dump(argv[2], argv[3]);
    if (mode == "pack") return cmd_pack(argv[2], argv[3]);
    cerr << "Unknown mode: " << mode << "\n";
    return 1;
}
