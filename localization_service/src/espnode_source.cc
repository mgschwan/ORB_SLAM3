#include "localization_service/espnode_source.h"
#include "localization_service/ingest_queue.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include <opencv2/imgcodecs.hpp>

namespace localization_service {

static constexpr uint8_t  PTYPE_IMAGE   = 0x01;
static constexpr uint8_t  PTYPE_IMU     = 0x02;
static constexpr uint8_t  PTYPE_TRIGGER = 0x03;
static constexpr int      TCP_PORT      = 11212;
static constexpr int      UDP_DISC_PORT = 11211;
static constexpr size_t   IMU_FRAME_BYTES = 6 * sizeof(float); // roll,pitch,yaw,vx,vy,vz

#pragma pack(push, 1)
struct TcpHeader {
    uint8_t  packet_type;
    uint32_t frame_time;   // ESP32 millis()
    uint32_t total_size;
};
#pragma pack(pop)

// ---- ImuBuffer ---------------------------------------------------------------

void ImuBuffer::push(const std::vector<ImuSample>& batch) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto& s : batch)
        samples_.push_back(s);
}

std::vector<ImuSample> ImuBuffer::drain() {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<ImuSample> out(samples_.begin(), samples_.end());
    samples_.clear();
    return out;
}

bool ImuBuffer::empty() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return samples_.empty();
}

// ---- EspnodeSource -----------------------------------------------------------

EspnodeSource::EspnodeSource(const std::string& ip, float fps)
    : ip_(ip), fps_(fps) {}

EspnodeSource::~EspnodeSource() { stop(); }

std::string EspnodeSource::discover(int timeoutMs) {
    if (!ip_.empty()) return ip_;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("espnode discover: socket"); return ""; }

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(UDP_DISC_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("espnode discover: bind");
        close(fd);
        return "";
    }

    struct timeval tv{ timeoutMs / 1000, (timeoutMs % 1000) * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::cout << "Waiting for ESP32 UDP broadcast on port " << UDP_DISC_PORT << "...\n";

    char buf[64]{};
    sockaddr_in sender{};
    socklen_t   slen = sizeof(sender);
    ssize_t     n    = recvfrom(fd, buf, sizeof(buf) - 1, 0,
                                reinterpret_cast<sockaddr*>(&sender), &slen);
    close(fd);

    if (n <= 0) {
        std::cerr << "ESP32 discovery timed out\n";
        return "";
    }

    buf[n] = '\0';
    ip_ = buf;
    while (!ip_.empty() && (ip_.back() == '\n' || ip_.back() == '\r' || ip_.back() == ' '))
        ip_.pop_back();

    std::cout << "Discovered ESP32 at " << ip_ << "\n";
    return ip_;
}

void EspnodeSource::start(IngestQueue& frameQueue, ImuBuffer& imuBuf) {
    if (ip_.empty()) {
        std::cerr << "EspnodeSource::start() called before discover()\n";
        return;
    }
    running_ = true;
    sessionThread_ = std::thread(&EspnodeSource::sessionLoop, this,
                                 std::ref(frameQueue), std::ref(imuBuf));
}

void EspnodeSource::stop() {
    running_ = false;
    if (sessionThread_.joinable()) sessionThread_.join();
}

bool EspnodeSource::recvAll(int fd, void* buf, size_t n) {
    auto* p    = static_cast<uint8_t*>(buf);
    size_t got = 0;
    while (got < n) {
        if (!running_) return false;
        ssize_t r = recv(fd, p + got, n - got, 0);
        if (r > 0) {
            got += static_cast<size_t>(r);
        } else if (r == 0) {
            return false;  // peer closed
        } else if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;      // SO_RCVTIMEO tick — retry
        } else {
            std::cerr << "espnode: recv error: " << strerror(errno) << "\n";
            return false;
        }
    }
    return true;
}

bool EspnodeSource::readPacket(int fd, Packet& out) {
    TcpHeader hdr;
    if (!recvAll(fd, &hdr, sizeof(hdr))) return false;

    out.type       = hdr.packet_type;
    out.esp_millis = hdr.frame_time;
    out.payload.resize(hdr.total_size);

    if (hdr.total_size > 0 && !recvAll(fd, out.payload.data(), hdr.total_size))
        return false;

    return true;
}

void EspnodeSource::rxLoop(int fd) {
    while (running_) {
        Packet pkt;
        if (!readPacket(fd, pkt)) break;
        std::lock_guard<std::mutex> lk(pktMtx_);
        pktQueue_.push_back(std::move(pkt));
    }
    rxDone_ = true;
}

void EspnodeSource::sessionLoop(IngestQueue& frameQueue, ImuBuffer& imuBuf) {
    using clock = std::chrono::steady_clock;
    const auto triggerInterval = std::chrono::duration<double>(1.0 / fps_);

    while (running_) {
        // ---- Connect ---------------------------------------------------
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            perror("espnode: socket");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        // Short timeout just for the connect() call.
        struct timeval tvConn{ 5, 0 };
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tvConn, sizeof(tvConn));

        sockaddr_in srv{};
        srv.sin_family = AF_INET;
        srv.sin_port   = htons(TCP_PORT);
        inet_pton(AF_INET, ip_.c_str(), &srv.sin_addr);

        std::cout << "Connecting to ESP32 at " << ip_ << ":" << TCP_PORT << "\n";
        if (connect(fd, reinterpret_cast<sockaddr*>(&srv), sizeof(srv)) != 0) {
            perror("espnode: connect");
            close(fd);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }
        std::cout << "ESP32 connected.\n";

        // TCP keepalive: detect a dead connection (e.g. WiFi drop on the device)
        // in ~14 seconds rather than waiting for the OS default (minutes).
        {
            int on = 1, idle = 5, intvl = 3, cnt = 3;
            setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE,  &on,    sizeof(on));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,  sizeof(idle));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,   sizeof(cnt));
        }

        // Switch to a 5-second recv timeout so the RX thread wakes up
        // periodically to check running_ if no packets arrive.
        struct timeval tvRecv{ 5, 0 };
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tvRecv, sizeof(tvRecv));

        // ---- Start RX thread -------------------------------------------
        rxDone_ = false;
        { std::lock_guard<std::mutex> lk(pktMtx_); pktQueue_.clear(); }
        std::thread rxThread(&EspnodeSource::rxLoop, this, fd);

        // ---- TX + dispatch loop ----------------------------------------
        // Send the first trigger immediately, then at fps_ rate.
        // pendingTriggers ensures we never queue more than one trigger at a
        // time: the next trigger is held until the previous frame arrives.
        auto lastTrigger    = clock::now() - triggerInterval;
        int  pendingTriggers = 0;

        while (running_ && !rxDone_) {
            // TX: fire trigger when due and no frame is already in flight.
            auto now = clock::now();
            if (pendingTriggers == 0 && now - lastTrigger >= triggerInterval) {
                uint8_t trig = PTYPE_TRIGGER;
                if (send(fd, &trig, 1, MSG_NOSIGNAL) != 1) {
                    std::cerr << "espnode: trigger send failed\n";
                    break;
                }
                ++pendingTriggers;
                lastTrigger = now;
            }

            // Drain the RX packet queue
            std::deque<Packet> batch;
            {
                std::lock_guard<std::mutex> lk(pktMtx_);
                std::swap(batch, pktQueue_);
            }

            for (auto& pkt : batch) {
                if (pkt.type == PTYPE_IMAGE) {
                    if (pendingTriggers > 0) --pendingTriggers;
                    if (pkt.payload.empty()) continue;

                    cv::Mat img = cv::imdecode(pkt.payload, cv::IMREAD_COLOR);
                    if (img.empty()) {
                        std::cerr << "espnode: imdecode failed ("
                                  << pkt.payload.size() << " bytes)\n";
                        continue;
                    }

                    IngestFrame frame;
                    frame.image     = std::move(img);
                    frame.timestamp = static_cast<double>(pkt.esp_millis);
                    frameQueue.push(std::move(frame));

                } else if (pkt.type == PTYPE_IMU) {
                    size_t nSamples = pkt.payload.size() / IMU_FRAME_BYTES;
                    if (nSamples == 0) continue;

                    std::vector<ImuSample> samples;
                    samples.reserve(nSamples);
                    for (size_t i = 0; i < nSamples; ++i) {
                        float v[6];
                        std::memcpy(v, pkt.payload.data() + i * IMU_FRAME_BYTES,
                                    IMU_FRAME_BYTES);
                        ImuSample s;
                        s.esp_millis = pkt.esp_millis;
                        s.roll  = v[0]; s.pitch = v[1]; s.yaw = v[2];
                        s.vx    = v[3]; s.vy    = v[4]; s.vz  = v[5];
                        samples.push_back(s);
                    }
                    imuBuf.push(samples);

                } else {
                    std::cerr << "espnode: unknown packet type "
                              << static_cast<int>(pkt.type) << "\n";
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // ---- Teardown --------------------------------------------------
        shutdown(fd, SHUT_RDWR);
        close(fd);
        rxThread.join();
        std::cout << "ESP32 session ended.\n";

        if (running_)
            std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

} // namespace localization_service
