#pragma once

#include <iostream>
#include <stdexcept>
#include <string>

#include "localization_service/config.h"

namespace localization_service {

struct ServiceArgs {
    std::string   vocabPath;
    std::string   settingsPath;
    std::string   cameraSource;
    bool          localizeOnly = false;
    unsigned long mapId        = 0;
    int           port         = LOCALIZATION_SERVICE_PORT;
    float         espnodeFps   = 10.0f;
};

inline void printUsage(const char* prog)
{
    std::cerr
        << "\nUsage: " << prog
        << " <vocab> <settings> <camera_source> [options]\n\n"
           "  camera_source:\n"
           "    <url>             V4L2 device (/dev/videoN or index), MJPEG/RTSP URL\n"
           "    none              receive frames via POST /api/frame\n"
           "    espnode           auto-discover ESP32 sensor node via UDP broadcast\n"
           "    espnode:<ip>      connect to ESP32 sensor node at the given IP\n\n"
           "  options:\n"
           "    --localize        start in localization-only mode\n"
           "    --map-id <n>      map index to activate on startup (default: 0)\n"
           "    --port <n>        HTTP control server port (default: "
        << LOCALIZATION_SERVICE_PORT << ")\n"
           "    --espnode-fps <n> frame trigger rate for espnode source (default: 10)\n\n";
}

// Parses argc/argv into out.
// Returns false and prints usage when required positional arguments are missing.
// Unknown flags emit a warning but are not fatal.
inline bool parseArgs(int argc, char** argv, ServiceArgs& out)
{
    if (argc < 4) {
        printUsage(argv[0]);
        return false;
    }

    out.vocabPath    = argv[1];
    out.settingsPath = argv[2];
    out.cameraSource = argv[3];

    for (int i = 4; i < argc; ++i) {
        const std::string a(argv[i]);

        auto requireNext = [&](const std::string& flag) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "error: " << flag << " requires a value\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (a == "--localize") {
            out.localizeOnly = true;
        } else if (a == "--map-id") {
            const char* v = requireNext(a);
            if (!v) return false;
            try { out.mapId = std::stoul(v); }
            catch (...) { std::cerr << "error: --map-id value is not a number\n"; return false; }
        } else if (a == "--port") {
            const char* v = requireNext(a);
            if (!v) return false;
            try { out.port = std::stoi(v); }
            catch (...) { std::cerr << "error: --port value is not a number\n"; return false; }
        } else if (a == "--espnode-fps") {
            const char* v = requireNext(a);
            if (!v) return false;
            try { out.espnodeFps = std::stof(v); }
            catch (...) { std::cerr << "error: --espnode-fps value is not a number\n"; return false; }
        } else {
            std::cerr << "warning: unknown argument '" << a << "' (ignored)\n";
        }
    }
    return true;
}

} // namespace localization_service
