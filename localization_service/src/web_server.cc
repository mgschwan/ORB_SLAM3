#include "localization_service/web_server.h"
#include "localization_service/calibration_manager.h"

#include <iostream>
#include <fstream>
#include <thread>
#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <opencv2/imgcodecs.hpp>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include <System.h>
#include <Atlas.h>
#include <Map.h>

namespace localization_service {

// ============================================================================
// Construction / destruction
// ============================================================================

WebServer::WebServer(ORB_SLAM3::System&  slam,
                     LifecycleFlags&     flags,
                     PoseState&          pose,
                     CalibrationManager& calib,
                     std::atomic<bool>&  localizationMode,
                     long unsigned int   initialMapId)
    : slam_(slam)
    , flags_(flags)
    , pose_(pose)
    , calib_(calib)
    , localizationMode_(localizationMode)
    , initialMapId_(initialMapId)
{}

WebServer::~WebServer()
{
    closeServer();
}

void WebServer::closeServer()
{
    if (serverFd_ >= 0) {
        close(serverFd_);
        serverFd_ = -1;
    }
}

// ============================================================================
// run() — accept loop
// ============================================================================

void WebServer::run()
{
    signal(SIGPIPE, SIG_IGN); // prevent SIGPIPE from crashing on broken SSE clients

    if (!setupSocket())
        return;

    std::cout << "\n--------------------------------------------------------\n"
              << "Remote control running at http://localhost:" << LOCALIZATION_SERVICE_PORT << "\n"
              << "Console commands: 'loc', 'map', 'quit', 'pause', 'resume'\n"
              << "--------------------------------------------------------\n\n";

    while (flags_.running) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(serverFd_, &readfds);

        struct timeval tv = {1, 0};
        if (select(serverFd_ + 1, &readfds, nullptr, nullptr, &tv) <= 0)
            continue;

        if (!FD_ISSET(serverFd_, &readfds))
            continue;

        int clientFd = accept(serverFd_, nullptr, nullptr);
        if (clientFd < 0)
            continue;

        handleConnection(clientFd);
    }

    closeServer();
}

bool WebServer::setupSocket()
{
    serverFd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd_ < 0) {
        std::cerr << "WebServer: socket creation failed\n";
        return false;
    }

    int opt = 1;
    setsockopt(serverFd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(LOCALIZATION_SERVICE_PORT);

    if (bind(serverFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "WebServer: bind failed on port " << LOCALIZATION_SERVICE_PORT << "\n";
        closeServer();
        return false;
    }

    listen(serverFd_, kTcpBacklog);
    return true;
}

// ============================================================================
// Per-connection handling
// ============================================================================

void WebServer::handleConnection(int clientFd)
{
    // Read until headers are complete (or limit exceeded).
    std::string rawRequest;
    char        buf[kHttpReadBufBytes];
    size_t      headerEnd = std::string::npos;

    while (true) {
        int n = read(clientFd, buf, sizeof(buf));
        if (n <= 0) break;
        rawRequest.append(buf, n);
        headerEnd = rawRequest.find("\r\n\r\n");
        if (headerEnd != std::string::npos) break;
        if (rawRequest.size() > static_cast<size_t>(kHttpMaxHeaderBytes)) break;
    }

    if (headerEnd == std::string::npos) {
        close(clientFd);
        return;
    }

    // Dispatch to the first matching route.
    std::string response;
    bool        socketConsumed = false;

    if      (routeCalibrate (rawRequest, clientFd, response, socketConsumed)) {}
    else if (routeStream    (rawRequest, clientFd, socketConsumed))           {}
    else if (routeMap       (rawRequest, response))                           {}
    else if (routeAtlas     (rawRequest, clientFd, rawRequest, headerEnd,
                              response, socketConsumed))                      {}
    else if (routeControl   (rawRequest, response))                           {}
    else if (routeStaticFile(rawRequest, response))                           {}
    else
        response = makeBadRequest();

    if (!socketConsumed) {
        if (response.empty())
            response = makeBadRequest();
        write(clientFd, response.c_str(), response.size());
        close(clientFd);
    }
}

// ============================================================================
// Route: /api/calibrate/*
// ============================================================================

bool WebServer::routeCalibrate(const std::string& req, int fd,
                                std::string& response, bool& socketConsumed)
{
    if (req.find("GET /api/calibrate/") == std::string::npos)
        return false;

    if (req.find("GET /api/calibrate/mode?enable=true") != std::string::npos) {
        calib_.enableMode(true, flags_.paused);
        response = makeOkText();
    } else if (req.find("GET /api/calibrate/mode?enable=false") != std::string::npos) {
        calib_.enableMode(false, flags_.paused);
        response = makeOkText();
    } else if (req.find("GET /api/calibrate/capture") != std::string::npos) {
        float sz = queryFloat(req, "size", -1.f);
        calib_.requestCapture(sz);
        response = makeOkText("Capture Requested");
    } else if (req.find("GET /api/calibrate/status") != std::string::npos) {
        auto s = calib_.getStatus();
        std::string json = "{\"count\":"    + std::to_string(s.count)
                         + ",\"last_success\":" + (s.lastSuccess ? "true" : "false")
                         + ",\"calibrating\":"  + (s.active      ? "true" : "false")
                         + "}";
        response = makeOkJson(json);
    } else if (req.find("GET /api/calibrate/image") != std::string::npos) {
        cv::Mat img = calib_.getLatestImage();
        if (img.empty()) {
            response = make404("No image yet");
        } else {
            std::vector<uchar> encoded;
            cv::imencode(".jpg", img, encoded);
            std::string data(encoded.begin(), encoded.end());
            response = "HTTP/1.1 200 OK\r\n"
                       "Content-Type: image/jpeg\r\n"
                       "Content-Length: " + std::to_string(data.size()) + "\r\n"
                       "Connection: close\r\n\r\n" + data;
        }
    } else if (req.find("GET /api/calibrate/compute") != std::string::npos) {
        response = makeOkJson(calib_.computeJson());
    } else if (req.find("GET /api/calibrate/apply") != std::string::npos) {
        response = makeOkJson(calib_.applyManualJson(
            queryDouble(req, "fx"), queryDouble(req, "fy"),
            queryDouble(req, "cx"), queryDouble(req, "cy"),
            queryDouble(req, "k1"), queryDouble(req, "k2"),
            queryDouble(req, "p1"), queryDouble(req, "p2"),
            queryDouble(req, "k3")));
    } else {
        return false; // unknown /api/calibrate/ sub-path
    }

    (void)fd;
    (void)socketConsumed;
    return true;
}

// ============================================================================
// Route: /api/stream/pose  (Server-Sent Events)
// ============================================================================

bool WebServer::routeStream(const std::string& req, int fd,
                             bool& socketConsumed)
{
    if (req.find("GET /api/stream/pose") == std::string::npos)
        return false;

    std::thread([this, fd]{ sseLoop(fd); }).detach();
    socketConsumed = true;
    return true;
}

void WebServer::sseLoop(int fd)
{
    const std::string headers =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "Access-Control-Allow-Origin: *\r\n\r\n";

    if (write(fd, headers.c_str(), headers.size()) < 0) {
        close(fd);
        return;
    }

    while (flags_.running) {
        auto snap = pose_.snapshot();
        std::string event;
        if (snap.valid) {
            event = "data: {\"valid\":true"
                    ",\"x\":"  + std::to_string(snap.x)  +
                    ",\"y\":"  + std::to_string(snap.y)  +
                    ",\"z\":"  + std::to_string(snap.z)  +
                    ",\"qx\":" + std::to_string(snap.qx) +
                    ",\"qy\":" + std::to_string(snap.qy) +
                    ",\"qz\":" + std::to_string(snap.qz) +
                    ",\"qw\":" + std::to_string(snap.qw) +
                    "}\n\n";
        } else {
            event = "data: {\"valid\":false}\n\n";
        }

        if (write(fd, event.c_str(), event.size()) < 0)
            break; // client disconnected

        usleep(kSseFrameDelayUs);
    }

    close(fd);
}

// ============================================================================
// Route: /api/map/*
// ============================================================================

bool WebServer::routeMap(const std::string& req, std::string& response)
{
    if (req.find("GET /api/map/") == std::string::npos)
        return false;

    if (req.find("GET /api/map/points") != std::string::npos) {
        ORB_SLAM3::Map* pMap = slam_.GetAtlas()->GetCurrentMap();
        std::string json = "[";
        if (pMap) {
            bool first = true;
            for (ORB_SLAM3::MapPoint* pMP : pMap->GetAllMapPoints()) {
                if (!pMP || pMP->isBad()) continue;
                if (!first) json += ",";
                Eigen::Vector3f p = pMP->GetWorldPos();
                json += "{\"x\":" + std::to_string(p.x())
                      + ",\"y\":" + std::to_string(p.y())
                      + ",\"z\":" + std::to_string(p.z()) + "}";
                first = false;
            }
        }
        json += "]";
        response = makeOkJson(json);

    } else if (req.find("GET /api/map/auto_align_floor") != std::string::npos) {
        response = handleAutoAlignFloor();

    } else if (req.find("GET /api/map/align_floor") != std::string::npos) {
        float pitch = queryFloat(req, "pitch");
        float roll  = queryFloat(req, "roll");
        response    = handleAlignFloor(pitch, roll);

    } else {
        return false;
    }

    return true;
}

std::string WebServer::handleAutoAlignFloor()
{
    ORB_SLAM3::Map* pMap = slam_.GetAtlas()->GetCurrentMap();
    if (!pMap)
        return makeOkJson("{\"success\":false,\"error\":\"no active map\"}");

    std::vector<Eigen::Vector3f> pts;
    for (auto* pMP : pMap->GetAllMapPoints()) {
        if (!pMP || pMP->isBad()) continue;
        pts.push_back(pMP->GetWorldPos());
    }
    if (pts.size() < 10)
        return makeOkJson("{\"success\":false,\"error\":\"not enough map points\"}");

    // Sort descending by Y (largest Y = physically lowest in ORB-SLAM3 convention)
    std::sort(pts.begin(), pts.end(),
        [](const Eigen::Vector3f& a, const Eigen::Vector3f& b){ return a.y() > b.y(); });
    const size_t nFloor = std::max(size_t{10}, pts.size() * 2 / 5);
    pts.resize(nFloor);

    // Estimate scale for adaptive inlier threshold
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (auto& p : pts) centroid += p;
    centroid /= static_cast<float>(nFloor);
    float spread = 0.f;
    for (auto& p : pts) spread += (p - centroid).norm();
    spread /= static_cast<float>(nFloor);
    const float inlierThresh = std::max(0.01f, spread * 0.05f);

    // RANSAC plane fit
    Eigen::Vector3f bestNormal(0.f, -1.f, 0.f);
    Eigen::Vector3f bestPoint = pts[0];
    int             bestInliers = 0;
    std::srand(12345);
    for (int iter = 0; iter < 300; ++iter) {
        int i0 = std::rand() % nFloor;
        int i1 = std::rand() % nFloor;
        int i2 = std::rand() % nFloor;
        if (i0 == i1 || i1 == i2 || i0 == i2) continue;
        Eigen::Vector3f n = (pts[i1]-pts[i0]).cross(pts[i2]-pts[i0]);
        if (n.norm() < 1e-6f) continue;
        n.normalize();
        if (n.y() > 0.f) n = -n;
        int inliers = 0;
        for (const auto& p : pts)
            if (std::abs(n.dot(p - pts[i0])) < inlierThresh) ++inliers;
        if (inliers > bestInliers) {
            bestInliers = inliers;
            bestNormal  = n;
            bestPoint   = pts[i0];
        }
    }
    (void)bestPoint;

    Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(
        bestNormal, Eigen::Vector3f(0.f, -1.f, 0.f));
    const float angle    = 2.f * std::acos(std::min(1.f, std::abs(q.w())));
    const float maxAngle = 45.f * static_cast<float>(M_PI) / 180.f;

    if (angle >= maxAngle) {
        std::cout << ">>> [Web] Auto floor alignment skipped: angle too large ("
                  << (angle * 180.f / M_PI) << " deg) <<<\n";
        return makeOkJson("{\"success\":false,\"error\":\"rotation too large\""
                          ",\"angle_deg\":" + std::to_string(angle * 180.f / M_PI) + "}");
    }

    Sophus::SE3f T(q.toRotationMatrix(), Eigen::Vector3f::Zero());
    pMap->ApplyScaledRotation(T, 1.f, false);
    std::cout << ">>> [Web] Auto floor alignment applied: "
              << bestInliers << "/" << nFloor
              << " inliers, angle=" << (angle * 180.f / M_PI) << " deg <<<\n";
    return makeOkJson("{\"success\":true"
                      ",\"inliers\":"   + std::to_string(bestInliers)
                    + ",\"total\":"     + std::to_string(nFloor)
                    + ",\"angle_deg\":" + std::to_string(angle * 180.f / M_PI) + "}");
}

std::string WebServer::handleAlignFloor(float pitch, float roll)
{
    ORB_SLAM3::Map* pMap = slam_.GetAtlas()->GetCurrentMap();
    if (!pMap)
        return makeOkJson("{\"success\":false,\"error\":\"no active map\"}");

    Eigen::Matrix3f R =
        (Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()) *
         Eigen::AngleAxisf(roll,  Eigen::Vector3f::UnitZ())).toRotationMatrix();
    pMap->ApplyScaledRotation(Sophus::SE3f(R, Eigen::Vector3f::Zero()), 1.f, false);
    std::cout << ">>> [Web] Floor alignment applied: pitch=" << pitch
              << " roll=" << roll << " rad <<<\n";
    return makeOkJson("{\"success\":true}");
}

// ============================================================================
// Route: /api/atlas/*
// ============================================================================

bool WebServer::routeAtlas(const std::string& req, int fd,
                            const std::string& rawRequest, size_t headerEnd,
                            std::string& response, bool& socketConsumed)
{
    if (req.find("/api/atlas/") == std::string::npos)
        return false;

    if (req.find("GET /api/atlas/download") != std::string::npos) {
        const bool wasMapping = !localizationMode_;
        if (wasMapping) {
            std::cout << ">>> [Web] Temporarily switching to localization mode for safe export...\n";
            slam_.ActivateLocalizationMode();
        }
        usleep(kAtlasSaveDelayUs);
        slam_.SaveAtlas(kExportAtlasPath, ORB_SLAM3::System::BINARY_FILE);
        if (wasMapping) {
            std::cout << ">>> [Web] Resuming mapping mode...\n";
            slam_.DeactivateLocalizationMode();
        }

        std::string filename = std::string(kExportAtlasPath) + ".osa";
        std::ifstream file(filename, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            response = make500("Failed to save atlas");
            return true;
        }
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::string headers =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/octet-stream\r\n"
            "Content-Disposition: attachment; filename=\"atlas.osa\"\r\n"
            "Content-Length: " + std::to_string(size) + "\r\n"
            "Connection: close\r\n\r\n";
        write(fd, headers.c_str(), headers.size());
        char chunk[8192];
        while (file.read(chunk, sizeof(chunk)) || file.gcount() > 0)
            write(fd, chunk, file.gcount());
        close(fd);
        socketConsumed = true;

    } else if (req.find("POST /api/atlas/upload") != std::string::npos) {
        response = handleAtlasUpload(fd, rawRequest, headerEnd);

    } else {
        return false;
    }

    return true;
}

std::string WebServer::handleAtlasUpload(int fd, const std::string& rawRequest,
                                          size_t headerEnd)
{
    if (rawRequest.find("Expect: 100-continue") != std::string::npos) {
        const std::string cont = "HTTP/1.1 100 Continue\r\n\r\n";
        write(fd, cont.c_str(), cont.size());
    }

    const size_t clPos = rawRequest.find("Content-Length: ");
    if (clPos == std::string::npos)
        return "HTTP/1.1 411 Length Required\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nLength Required";

    const size_t clEnd      = rawRequest.find("\r\n", clPos);
    const long   totalBytes = std::stol(rawRequest.substr(clPos + 16, clEnd - (clPos + 16)));
    std::cout << ">>> [Web] Receiving atlas upload (" << totalBytes << " bytes)...\n";

    const std::string bodyPrefix = rawRequest.substr(headerEnd + 4);
    long remaining = totalBytes - static_cast<long>(bodyPrefix.size());

    std::ofstream out(std::string(kImportAtlasPath) + ".osa", std::ios::binary);
    if (!bodyPrefix.empty())
        out.write(bodyPrefix.data(), bodyPrefix.size());

    char chunk[kUploadChunkBytes];
    while (remaining > 0) {
        int n = read(fd, chunk, std::min(static_cast<long>(sizeof(chunk)), remaining));
        if (n <= 0) {
            std::cerr << ">>> [Web] Upload interrupted. Remaining: " << remaining << "\n";
            break;
        }
        out.write(chunk, n);
        remaining -= n;
        if (totalBytes > 1024*1024 && (remaining % (5*1024*1024) < kUploadChunkBytes))
            std::cout << ">>> [Web] Progress: "
                      << (100 * (totalBytes - remaining) / totalBytes) << "%\n";
    }
    out.close();

    std::cout << ">>> [Web] Loading uploaded atlas...\n";
    flags_.paused = true;
    if (slam_.LoadAtlas(kImportAtlasPath, ORB_SLAM3::System::BINARY_FILE)) {
        std::cout << ">>> [Web] Atlas loaded successfully <<<\n";
        return makeOkText();
    }
    std::cerr << ">>> [Web] Failed to load uploaded atlas <<<\n";
    return make500("Load failed");
}

// ============================================================================
// Route: /api/status  and legacy single-word control paths
// ============================================================================

bool WebServer::routeControl(const std::string& req, std::string& response)
{
    if (req.find("GET /api/status") != std::string::npos) {
        long unsigned int currentMapId = 0;
        ORB_SLAM3::Map* pMap = slam_.GetAtlas()->GetCurrentMap();
        if (pMap) currentMapId = pMap->GetId();

        std::string json = "{";
        json += "\"localizationMode\":"  + std::string(localizationMode_ ? "true" : "false") + ",";
        json += "\"paused\":"            + std::string(flags_.paused     ? "true" : "false") + ",";
        json += "\"currentMapId\":"      + std::to_string(currentMapId)  + ",";
        json += "\"maps\":[";
        bool first = true;
        for (ORB_SLAM3::Map* m : slam_.GetAtlas()->GetAllMaps()) {
            if (!m) continue;
            if (!first) json += ",";
            json += "{\"id\":"        + std::to_string(m->GetId())
                  + ",\"keyframes\":" + std::to_string(m->KeyFramesInMap())
                  + ",\"mappoints\":" + std::to_string(m->MapPointsInMap()) + "}";
            first = false;
        }
        json += "]}";
        response = makeOkJson(json);

    } else if (req.find("GET /loc ") != std::string::npos) {
        if (!localizationMode_) {
            localizationMode_ = true;
            slam_.GetAtlas()->SwitchToMap(initialMapId_);
            slam_.ActivateLocalizationMode();
            slam_.ForceRelocalization();
            std::cout << ">>> [Web] Switched to Localization Mode <<<\n";
        }
        response = makeOkText();

    } else if (req.find("GET /map ") != std::string::npos) {
        if (localizationMode_) {
            localizationMode_ = false;
            slam_.DeactivateLocalizationMode();
            std::cout << ">>> [Web] Switched to Mapping Mode <<<\n";
        }
        response = makeOkText();

    } else if (req.find("GET /pause ") != std::string::npos) {
        flags_.paused = true;
        std::cout << ">>> [Web] Paused Processing <<<\n";
        response = makeOkText();

    } else if (req.find("GET /resume ") != std::string::npos) {
        flags_.paused = false;
        std::cout << ">>> [Web] Resumed Processing <<<\n";
        response = makeOkText();

    } else if (req.find("GET /switchmap?id=") != std::string::npos) {
        const std::string prefix = "GET /switchmap?id=";
        size_t pos = req.find(prefix);
        size_t end = req.find(" HTTP", pos);
        if (end != std::string::npos) {
            long unsigned int id = std::stoul(req.substr(pos + prefix.size(),
                                                          end - (pos + prefix.size())));
            slam_.SwitchToMap(id);
            if (localizationMode_) slam_.ForceRelocalization();
            std::cout << ">>> [Web] Switched to Map ID: " << id << " <<<\n";
        }
        response = makeOkText();

    } else if (req.find("GET /newmap ") != std::string::npos) {
        slam_.GetAtlas()->CreateNewMap();
        long unsigned int id = slam_.GetAtlas()->GetCurrentMap()->GetId();
        slam_.SwitchToMap(id);
        std::cout << ">>> [Web] Created and switched to New Map ID: " << id << " <<<\n";
        response = makeOkText();

    } else {
        return false;
    }

    return true;
}

// ============================================================================
// Route: static files from html/
// ============================================================================

bool WebServer::routeStaticFile(const std::string& req, std::string& response)
{
    const size_t start = req.find("GET /");
    if (start == std::string::npos) return false;

    const size_t pathStart = start + 5;
    const size_t pathEnd   = req.find(" HTTP", pathStart);
    if (pathEnd == std::string::npos) return false;

    std::string path = req.substr(pathStart, pathEnd - pathStart);
    if (path.empty()) path = "index.html";

    // Prevent directory traversal
    if (path.find("..") != std::string::npos
     || path.find("//") != std::string::npos
     || path[0] == '/') {
        response = "HTTP/1.1 403 Forbidden\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nForbidden";
        return true;
    }

    const std::string fullPath = std::string(kStaticFileRoot) + path;
    std::ifstream file(fullPath, std::ios::binary);
    if (!file.is_open()) {
        response = make404("File not found: " + fullPath);
        return true;
    }

    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
    response = makeOkResponse(mimeType(path), content);
    return true;
}

// ============================================================================
// HTTP utilities
// ============================================================================

std::string WebServer::makeOkResponse(const std::string& contentType,
                                       const std::string& body)
{
    return "HTTP/1.1 200 OK\r\n"
           "Content-Type: " + contentType + "\r\n"
           "Connection: close\r\n\r\n" + body;
}

std::string WebServer::makeOkJson(const std::string& json)
{
    return makeOkResponse("application/json", json);
}

std::string WebServer::makeOkText(const std::string& text)
{
    return makeOkResponse("text/plain", text);
}

std::string WebServer::make404(const std::string& detail)
{
    return "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n" + detail;
}

std::string WebServer::make500(const std::string& detail)
{
    return "HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n" + detail;
}

std::string WebServer::makeBadRequest()
{
    return "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nBad Request";
}

std::string WebServer::mimeType(const std::string& path)
{
    if (path.find(".css") != std::string::npos)  return "text/css";
    if (path.find(".js")  != std::string::npos)  return "application/javascript";
    if (path.find(".png") != std::string::npos)  return "image/png";
    if (path.find(".jpg") != std::string::npos)  return "image/jpeg";
    return "text/html";
}

// ============================================================================
// Query-string helpers
// ============================================================================

std::string WebServer::queryParam(const std::string& req, const std::string& key)
{
    const std::string needle = key + "=";
    size_t pos = req.find(needle);
    if (pos == std::string::npos) return {};
    pos += needle.size();
    size_t end = req.find_first_of(" &\r\n", pos);
    return req.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
}

double WebServer::queryDouble(const std::string& req, const std::string& key,
                               double fallback)
{
    const std::string v = queryParam(req, key);
    if (v.empty()) return fallback;
    try { return std::stod(v); } catch (...) { return fallback; }
}

float WebServer::queryFloat(const std::string& req, const std::string& key,
                             float fallback)
{
    const std::string v = queryParam(req, key);
    if (v.empty()) return fallback;
    try { return std::stof(v); } catch (...) { return fallback; }
}

} // namespace localization_service
