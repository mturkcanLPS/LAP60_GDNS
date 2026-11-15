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
 * Otopilottan gelen:
 *   0x4A (15_11, OKAD V1.1.3): 14 B yuk = float ekfZ + uint8 gecerli + float dronBaslik +
 *        float yerBaslik + uint8 baslikDurum. Header BB 4A 0E ED. Eski format asagida:
 *   0x4A EKF YUKSEKLIGI, 5 B yuk: float ekfZ [m] (little-endian) + uint8 gecerli
 *        (1: EKF dikey kanal hazir). Header BB 4A 05 F6. GDNS EKF'i baro + LRF +
 *        bizim gonderdigimiz HRVS'i birlestirir -> HRVS kopsa da yukseklik gelir.
 *
 * Bayt akisi parca parca gelebilir; ayristirici her bayti tamponlar, header ya da
 * yuk checksum'i tutmazsa SADECE bir bayt atlayip yeniden senkron arar (bozuk bir
 * cercevenin icinden baslayan gecerli cerceve kaybolmaz).
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <vector>

namespace lps {

constexpr uint8_t AP_SYNC_BYTE   = 0xBB;
constexpr uint8_t AP_MSG_POWEROFF = 0x49;
constexpr uint8_t AP_MSG_EKF_ALT  = 0x4A;

/** 0x4A icerigi. OKAD V1.1.3 (15_11): 14 B yuk = ekfZ + gecerli + dron/yer basligi + durum.
 *  Eski 5 B yuk (sadece ekfZ + gecerli) de kabul edilir; o zaman has_heading = false. */
struct EkfAltMsg {
    float   z = 0.0f;            // GDNS EKF yuksekligi [m]
    bool    valid = false;       // EKF dikey kanal hazir
    bool    has_heading = false; // 14 B format mi
    float   drone_hdg = 0.0f;    // dron gercek basligi [derece] 0-360, kuzey 0, saat yonu +
    float   ground_hdg = 0.0f;   // yer (arac) VN-200 basligi [derece]
    uint8_t status = 0;          // bit0-1 dron INS modu, bit2-3 yer INS modu, bit4 yer RTK taze
    int  droneIns() const { return status & 0x3; }
    int  groundIns() const { return (status >> 2) & 0x3; }
    bool groundFresh() const { return (status & 0x10) != 0; }
    /** Dron basligi - arac basligi, (-180, 180]. Saat yonu +. */
    float relHeading() const {
        float d = std::fmod(drone_hdg - ground_hdg, 360.0f);
        if (d > 180.0f) d -= 360.0f;
        if (d <= -180.0f) d += 360.0f;
        return d;
    }
};

inline bool decodeEkfAltMsg(const std::vector<uint8_t> &p, EkfAltMsg &m)
{
    if (p.size() != 5 && p.size() != 14) return false;
    float z;
    std::memcpy(&z, p.data(), sizeof(z));   // Jetson ve otopilot little-endian
    if (!std::isfinite(z)) return false;
    m = EkfAltMsg{};
    m.z = z;
    m.valid = p[4] == 1;
    if (p.size() == 14) {
        float dh, gh;
        std::memcpy(&dh, p.data() + 5, sizeof(dh));
        std::memcpy(&gh, p.data() + 9, sizeof(gh));
        m.status = p[13];
        m.has_heading = std::isfinite(dh) && std::isfinite(gh);
        if (m.has_heading) { m.drone_hdg = dh; m.ground_hdg = gh; }
    }
    return true;
}

/** Geriye uyum (testler / eski cagri): sadece yukseklik. */
inline bool decodeEkfAlt(const std::vector<uint8_t> &p, float &z_m, bool &valid)
{
    EkfAltMsg m;
    if (!decodeEkfAltMsg(p, m)) return false;
    z_m = m.z; valid = m.valid;
    return true;
}

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
