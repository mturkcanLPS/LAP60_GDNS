/**
 * autopilot_rx.hpp — otopilot -> Jetson seri mesaj ayristirici (ROS'tan bagimsiz).
 *
 * Cerceve bizim gonderdigimizle AYNI kural (0x42 / 0x48):
 *   [0xBB][msg_id][payloadSize][headerchk] [payload ...] [chksum]
 *   headerchk = ~(ilk 3 baytin toplami % 256) + 1
 *   chksum    = ~(payload toplami % 256) + 1
 *
 * Otopilottan gelen (02_11):
 *   0x49 KAPAT, 1 B yuk: 01 = Jetson'i kapat  ->  BB 49 01 FB 01 FF
 *
 * Bayt akisi parca parca gelebilir; ayristirici her bayti tamponlar, header ya da
 * yuk checksum'i tutmazsa SADECE bir bayt atlayip yeniden senkron arar (bozuk bir
 * cercevenin icinden baslayan gecerli cerceve kaybolmaz).
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <vector>

namespace lps {

constexpr uint8_t AP_SYNC_BYTE   = 0xBB;
constexpr uint8_t AP_MSG_POWEROFF = 0x49;

inline uint8_t apChecksum(const uint8_t *p, size_t n)
{
    uint32_t s = 0;
    for (size_t i = 0; i < n; ++i) s += p[i];
    return static_cast<uint8_t>(~(s % 256) + 1);
}

class AutopilotRxParser {
public:
    using FrameCb = std::function<void(uint8_t msg_id, const std::vector<uint8_t> &payload)>;

    explicit AutopilotRxParser(FrameCb cb) : cb_(std::move(cb)) {}

    void feed(const uint8_t *data, size_t n)
    {
        buf_.insert(buf_.end(), data, data + n);
        parse();
    }

    /* Hat bir sure sessizse cagrilir: bastaki yarim cerceve artik tamamlanmayacak.
       Gurultude checksum'i tesadufen tutan bir header (1/256) buyuk payloadSize
       soyleyip arkasindaki GERCEK cerceveyi tamponda bekletebilir — onu atip
       kalan baytlari yeniden tarar. */
    void idle()
    {
        while (!buf_.empty()) { dropOne(); parse(); }
    }

    uint64_t frames() const { return frames_; }
    uint64_t badHeader() const { return bad_header_; }
    uint64_t badPayload() const { return bad_payload_; }
    uint64_t skipped() const { return skipped_; }

private:
    void parse()
    {
        for (;;) {
            while (!buf_.empty() && buf_.front() != AP_SYNC_BYTE) { buf_.pop_front(); ++skipped_; }
            if (buf_.size() < 4) return;
            const uint8_t hdr[3] = {buf_[0], buf_[1], buf_[2]};
            if (apChecksum(hdr, 3) != buf_[3]) { dropOne(); ++bad_header_; continue; }
            const size_t len = 4 + (size_t)buf_[2] + 1;
            if (buf_.size() < len) return;
            std::vector<uint8_t> payload(buf_.begin() + 4, buf_.begin() + 4 + buf_[2]);
            if (apChecksum(payload.data(), payload.size()) != buf_[len - 1]) {
                dropOne(); ++bad_payload_; continue;
            }
            const uint8_t id = buf_[1];
            buf_.erase(buf_.begin(), buf_.begin() + (std::ptrdiff_t)len);
            ++frames_;
            cb_(id, payload);
        }
    }

    void dropOne() { buf_.pop_front(); ++skipped_; }

    FrameCb cb_;
    std::deque<uint8_t> buf_;
    uint64_t frames_ = 0, bad_header_ = 0, bad_payload_ = 0, skipped_ = 0;
};

}  // namespace lps
