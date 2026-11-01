/**
 * system_health.hpp — Jetson sistem sagligi ornekleyicisi (otopilot paketindeki health alani)
 *
 * Amac (2026-10-20): ucusta sistem sagligini UZAKTAN gormek. 19_10 ucusunda RAM
 * dolmasi (car_detector OOM), 70 C uyari esigi asan sicaklik, asiri akim (OC)
 * kismalari ve beklenmedik guc kesintisi ancak ucus sonrasi loglardan anlasilabildi.
 *
 *  - Root gerekmez; yalniz sysfs / procfs okur, toplam < 1 ms. ~1 Hz cagrilir.
 *  - hwmon / thermal_zone NUMARALARI acilistan acilisa degisebilir: yollar kurulumda
 *    ADA gore bulunur (cpu-thermal, gpu-thermal, tj-thermal, pwm_tach, ina3221
 *    VDD_IN, soctherm_oc).
 *  - Okunamayan alan: sicaklik INT8_MIN (-128), digerleri 0.
 *  - CPU yuzdesi /proc/stat'in ONCEKI cagriya gore farkindan hesaplanir: ilk cagri 0.
 */
#pragma once

#include <sys/statvfs.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace lps {

struct HealthSample {
    uint32_t uptime_s     = 0;     // Jetson acik kalma suresi
    uint8_t  cpu_load     = 0;     // % tum cekirdekler
    uint8_t  cpu_core_max = 0;     // % en yuklu cekirdek
    uint8_t  gpu_load     = 0;     // %
    int8_t   cpu_temp     = std::numeric_limits<int8_t>::min();   // C
    int8_t   gpu_temp     = std::numeric_limits<int8_t>::min();
    int8_t   tj_temp      = std::numeric_limits<int8_t>::min();   // en sicak nokta
    uint8_t  ram_used     = 0;     // % (MemTotal - MemAvailable)
    uint8_t  swap_used    = 0;     // %
    uint8_t  disk_used    = 0;     // % (kayit klasorunun dosya sistemi)
    uint16_t fan_rpm      = 0;
    uint16_t vdd_in_mv    = 0;     // modul girisi gerilimi (~5000 mV)
    uint16_t vdd_in_mw    = 0;     // modul girisi gucu
    uint16_t oc_events    = 0;     // asiri akim olaylari, acilistan beri (oc1+oc2+oc3)
};

class SystemHealth
{
public:
    explicit SystemHealth(std::string disk_path) : disk_path_(std::move(disk_path))
    {
        namespace fs = std::filesystem;
        std::error_code ec;
        for (const auto &z : fs::directory_iterator("/sys/class/thermal", ec)) {
            const std::string type = readStr(z.path() / "type");
            const std::string temp = (z.path() / "temp").string();
            if (type == "cpu-thermal") cpu_temp_ = temp;
            else if (type == "gpu-thermal") gpu_temp_ = temp;
            else if (type == "tj-thermal") tj_temp_ = temp;
        }
        for (const auto &h : fs::directory_iterator("/sys/class/hwmon", ec)) {
            const std::string name = readStr(h.path() / "name");
            if (name == "pwm_tach") {
                fan_rpm_ = (h.path() / "rpm").string();
            } else if (name == "soctherm_oc") {
                for (int i = 1; i <= 3; ++i)
                    oc_.push_back((h.path() / ("oc" + std::to_string(i) + "_event_cnt")).string());
            } else if (name == "ina3221") {
                for (int i = 1; i <= 3; ++i) {
                    if (readStr(h.path() / ("in" + std::to_string(i) + "_label")) == "VDD_IN") {
                        in_mv_ = (h.path() / ("in" + std::to_string(i) + "_input")).string();
                        in_ma_ = (h.path() / ("curr" + std::to_string(i) + "_input")).string();
                    }
                }
            }
        }
        for (const char *p : {"/sys/devices/platform/bus@0/17000000.gpu/load",
                              "/sys/devices/platform/17000000.gpu/load",
                              "/sys/devices/gpu.0/load"}) {
            if (fs::exists(p, ec)) { gpu_load_ = p; break; }
        }
    }

    /** Hangi kaynaklarin bulundugu (baslangic logu icin). */
    std::string sources() const
    {
        auto ok = [](const std::string &s) { return s.empty() ? "YOK" : "ok"; };
        std::ostringstream o;
        o << "cpu_temp=" << ok(cpu_temp_) << " gpu_temp=" << ok(gpu_temp_)
          << " tj=" << ok(tj_temp_) << " gpu_load=" << ok(gpu_load_)
          << " fan=" << ok(fan_rpm_) << " vdd_in=" << ok(in_mv_)
          << " oc=" << (oc_.empty() ? "YOK" : "ok") << " disk=" << disk_path_;
        return o.str();
    }

    HealthSample sample()
    {
        HealthSample s;
        double up = 0;
        if (std::FILE *f = std::fopen("/proc/uptime", "r")) {
            if (std::fscanf(f, "%lf", &up) != 1) up = 0;
            std::fclose(f);
        }
        s.uptime_s = (uint32_t)up;

        cpuLoad(s);
        s.gpu_load = clampU8(readLong(gpu_load_, 0) / 10);        // promil -> %
        s.cpu_temp = tempC(cpu_temp_);
        s.gpu_temp = tempC(gpu_temp_);
        s.tj_temp  = tempC(tj_temp_);
        memory(s);

        struct statvfs v{};
        if (!disk_path_.empty() && statvfs(disk_path_.c_str(), &v) == 0) {
            // df'in Use% hesabi: kullanilan / (kullanilan + kullaniciya bos); root payi haric
            const double used = (double)(v.f_blocks - v.f_bfree), avail = (double)v.f_bavail;
            if (used + avail > 0) s.disk_used = clampU8(100.0 * used / (used + avail));
        }

        s.fan_rpm = clampU16(readLong(fan_rpm_, 0));
        const long mv = readLong(in_mv_, 0), ma = readLong(in_ma_, 0);
        s.vdd_in_mv = clampU16(mv);
        s.vdd_in_mw = clampU16(mv * ma / 1000);
        long oc = 0;
        for (const auto &p : oc_) oc += readLong(p, 0);
        s.oc_events = clampU16(oc);
        return s;
    }

private:
    static std::string readStr(const std::filesystem::path &p)
    {
        std::ifstream f(p);
        std::string s;
        std::getline(f, s);
        return s;
    }
    static long readLong(const std::string &p, long dflt)
    {
        if (p.empty()) return dflt;
        std::ifstream f(p);
        long v;
        return (f >> v) ? v : dflt;
    }
    static uint8_t clampU8(double v) { return (uint8_t)std::clamp(v + 0.5, 0.0, 255.0); }
    static uint16_t clampU16(long v) { return (uint16_t)std::clamp(v, 0L, 65535L); }
    static int8_t tempC(const std::string &p)
    {
        const long m = readLong(p, std::numeric_limits<long>::min());   // mili-C
        if (m == std::numeric_limits<long>::min()) return std::numeric_limits<int8_t>::min();
        return (int8_t)std::clamp((m + 500) / 1000, -127L, 127L);
    }

    // /proc/stat: toplam ve cekirdek basina mesgul orani, onceki cagriya gore
    void cpuLoad(HealthSample &s)
    {
        std::ifstream f("/proc/stat");
        std::string line;
        std::vector<std::pair<uint64_t, uint64_t>> cur;   // (toplam, bos); [0] = hepsi
        while (std::getline(f, line) && line.rfind("cpu", 0) == 0) {
            std::istringstream is(line);
            std::string tag;
            uint64_t v, tot = 0, idle = 0;
            is >> tag;
            for (int i = 0; is >> v; ++i) {
                tot += v;
                if (i == 3 || i == 4) idle += v;   // idle + iowait
            }
            cur.emplace_back(tot, idle);
        }
        if (prev_cpu_.size() == cur.size()) {
            auto busy = [&](size_t i) {
                const double dt = (double)(cur[i].first - prev_cpu_[i].first);
                const double di = (double)(cur[i].second - prev_cpu_[i].second);
                return dt > 0 ? 100.0 * (1.0 - di / dt) : 0.0;
            };
            s.cpu_load = clampU8(busy(0));
            double mx = 0;
            for (size_t i = 1; i < cur.size(); ++i) mx = std::max(mx, busy(i));
            s.cpu_core_max = clampU8(mx);
        }
        prev_cpu_ = std::move(cur);
    }

    static void memory(HealthSample &s)
    {
        std::ifstream f("/proc/meminfo");
        std::string key;
        long val, total = 0, avail = 0, stotal = 0, sfree = 0;
        std::string unit;
        while (f >> key >> val) {
            std::getline(f, unit);
            if (key == "MemTotal:") total = val;
            else if (key == "MemAvailable:") avail = val;
            else if (key == "SwapTotal:") stotal = val;
            else if (key == "SwapFree:") sfree = val;
        }
        if (total > 0) s.ram_used = clampU8(100.0 * (double)(total - avail) / (double)total);
        if (stotal > 0) s.swap_used = clampU8(100.0 * (double)(stotal - sfree) / (double)stotal);
    }

    std::string disk_path_;
    std::string cpu_temp_, gpu_temp_, tj_temp_, gpu_load_, fan_rpm_, in_mv_, in_ma_;
    std::vector<std::string> oc_;
    std::vector<std::pair<uint64_t, uint64_t>> prev_cpu_;
};

}  // namespace lps
