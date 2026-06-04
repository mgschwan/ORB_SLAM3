#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace localization_service {

class IngestQueue;

// One fused IMU sample as produced by the ESP32's accIntegral library.
struct ImuSample {
    uint32_t esp_millis;         // ESP32 millis() at batch flush time
    float roll, pitch, yaw;      // radians
    float vx, vy, vz;            // mm/s
};

// Thread-safe accumulator for IMU samples arriving between frames.
// The tracking loop drains this once per frame.
class ImuBuffer {
public:
    void push(const std::vector<ImuSample>& batch);

    // Returns all queued samples and clears the buffer.
    std::vector<ImuSample> drain();

    bool empty() const;

private:
    mutable std::mutex    mtx_;
    std::deque<ImuSample> samples_;
};

// Manages the persistent TCP session with one ESP32-S3 orblsammer_espnode.
//
// Protocol (binary, little-endian):
//   ESP32 → host: PACKET_TYPE_IMAGE (0x01) — 9-byte header + JPEG bytes
//   ESP32 → host: PACKET_TYPE_IMU   (0x02) — 9-byte header + N×6×float32
//   host  → ESP32: PACKET_TYPE_TRIGGER (0x03) — single byte, no header
//
// Internally runs two threads per connection:
//   - RX thread : blocking reads with SO_RCVTIMEO; pushes packets to pktQueue_
//   - session thread: outer reconnect loop; fires triggers at fps_ rate;
//     dispatches IMAGE → IngestQueue, IMU → ImuBuffer
class EspnodeSource {
public:
    // ip = "" → auto-discover via UDP broadcast on port 11211
    // fps     → TRIGGER rate sent to the device (controls frame capture rate)
    EspnodeSource(const std::string& ip, float fps);
    ~EspnodeSource();

    // Block until the device broadcasts its IP (or ip_ is already set).
    // Returns the IP string, or "" on timeout.
    std::string discover(int timeoutMs = 15000);

    // Start the background session. discover() must have succeeded first.
    void start(IngestQueue& frameQueue, ImuBuffer& imuBuf);

    // Signal stop and block until the session thread exits.
    void stop();

    const std::string& ip() const { return ip_; }

private:
    struct Packet {
        uint8_t              type;
        uint32_t             esp_millis;
        std::vector<uint8_t> payload;
    };

    // Receive exactly n bytes, respecting running_ and EAGAIN retries.
    bool recvAll(int fd, void* buf, size_t n);

    // Read one header+payload packet from fd into out.
    bool readPacket(int fd, Packet& out);

    // RX thread: reads packets from fd, pushes to pktQueue_, sets rxDone_.
    void rxLoop(int fd);

    // Session thread: outer reconnect loop; TX triggers; dispatches packets.
    void sessionLoop(IngestQueue& frameQueue, ImuBuffer& imuBuf);

    std::string       ip_;
    float             fps_;

    std::atomic<bool> running_{false};
    std::thread       sessionThread_;

    // Shared between session thread (consumer) and per-connection RX thread (producer).
    std::mutex          pktMtx_;
    std::deque<Packet>  pktQueue_;
    std::atomic<bool>   rxDone_{false};
};

} // namespace localization_service
