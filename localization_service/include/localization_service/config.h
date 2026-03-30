#pragma once

// Network port the HTTP control server binds to.
// Used as both a build-time constant and in htons(), so a #define is appropriate.
#define LOCALIZATION_SERVICE_PORT 11142

namespace localization_service {

constexpr const char* kStaticFileRoot   = "html/";
constexpr const char* kExportAtlasPath  = "web_export";
constexpr const char* kImportAtlasPath  = "web_import";

constexpr int kSseFrameDelayUs    = 33000;   // ~30 fps
constexpr int kPauseSleepUs       = 100000;
constexpr int kCalibSleepUs       = 30000;
constexpr int kAtlasSaveDelayUs   = 100000;
constexpr int kTcpBacklog         = 3;
constexpr int kHttpMaxHeaderBytes = 16384;
constexpr int kHttpReadBufBytes   = 4096;
constexpr int kUploadChunkBytes   = 65536;

} // namespace localization_service
