#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "localization_service/config.h"
#include "localization_service/slam_state.h"

namespace ORB_SLAM3 { class System; }

namespace localization_service {

class CalibrationManager;

// Single-threaded HTTP/1.1 server.
//
// Each accepted connection is handled synchronously.
// The SSE pose stream and atlas download write directly to the socket and
// transfer ownership of the file descriptor; all other routes return a
// complete response string that is written and then closed by run().
//
// Call run() from a dedicated std::thread. It blocks until
// flags.running becomes false.
class WebServer {
public:
    WebServer(ORB_SLAM3::System&  slam,
              LifecycleFlags&     flags,
              PoseState&          pose,
              CalibrationManager& calib,
              std::atomic<bool>&  localizationMode,
              long unsigned int   initialMapId,
              std::string         staticFileRoot = kStaticFileRoot);

    ~WebServer();

    // Blocking accept loop. Returns when flags_.running is false.
    void run();

private:
    // ---- Socket lifecycle --------------------------------------------------
    bool setupSocket();        // bind + listen; returns false on error
    void closeServer();

    // ---- Per-connection handling -------------------------------------------
    void handleConnection(int clientFd);

    // ---- Route handlers ----------------------------------------------------
    // Each handler returns true if the route matched.
    // response is set to the complete HTTP response to write (may be empty if
    // the handler consumed the socket directly and set socketConsumed = true).

    bool routeCalibrate (const std::string& req, int fd,
                         std::string& response, bool& socketConsumed);

    bool routeStream    (const std::string& req, int fd,
                         bool& socketConsumed);

    bool routeMap       (const std::string& req,
                         std::string& response);
    std::string handleAutoAlignFloor();
    std::string handleAlignFloor(float pitch, float roll);

    bool routeAtlas     (const std::string& req, int fd,
                         const std::string& rawRequest, size_t headerEnd,
                         std::string& response, bool& socketConsumed);

    bool routeControl   (const std::string& req,
                         std::string& response);

    bool routeStaticFile(const std::string& req,
                         std::string& response);

    // ---- SSE helper --------------------------------------------------------
    // Runs in a detached thread. Takes ownership of fd (calls close(fd) on exit).
    void sseLoop(int fd);

    // ---- Atlas upload helper -----------------------------------------------
    std::string handleAtlasUpload(int fd, const std::string& rawRequest,
                                  size_t headerEnd);

    // ---- HTTP utilities ----------------------------------------------------
    static std::string makeOkResponse (const std::string& contentType,
                                        const std::string& body);
    static std::string makeOkJson     (const std::string& json);
    static std::string makeOkText     (const std::string& text = "OK");
    static std::string make404        (const std::string& detail = "Not Found");
    static std::string make500        (const std::string& detail = "Internal Server Error");
    static std::string makeBadRequest ();
    static std::string mimeType       (const std::string& path);

    // ---- Query-string helpers ----------------------------------------------
    static std::string queryParam  (const std::string& req, const std::string& key);
    static double      queryDouble (const std::string& req, const std::string& key,
                                    double fallback = 0.0);
    static float       queryFloat  (const std::string& req, const std::string& key,
                                    float fallback = 0.0f);

    // ---- Members -----------------------------------------------------------
    ORB_SLAM3::System&  slam_;
    LifecycleFlags&     flags_;
    PoseState&          pose_;
    CalibrationManager& calib_;
    std::atomic<bool>&  localizationMode_;
    long unsigned int   initialMapId_;
    std::string         staticFileRoot_;
    int                 serverFd_{-1};
};

} // namespace localization_service
