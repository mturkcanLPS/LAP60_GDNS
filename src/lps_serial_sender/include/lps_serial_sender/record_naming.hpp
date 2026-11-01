/**
 * record_naming.hpp — kayit dosya adlari: <onek>[_sNNNN]_YYYYmmdd_HHMMSS[_k].<uzanti>
 *
 * Neden: Jetson acilista saati son kaydedilen zamana geri yukluyor (sahada NTP
 * yok) -> her acilis AYNI tarihli adlari uretiyordu; 20_10'da yeni acilis 19_10
 * ucus kayitlarinin USTUNE yazdi. Sira ve tekillik artik tarihten degil:
 *  - sNNNN: launch'in verdigi oturum numarasi (servis her acilista +1, diske
 *    fsync'li; bkz. lps_bringup bringup.launch.py next_record_session). Ada gore
 *    siralama = kronolojik sira; bir oturumun tum dosyalari ayni numarayi tasir.
 *    0 -> numarasiz (node elle calistirildiginda).
 *  - Ad O_CREAT|O_EXCL ile ATOMIK sahiplenilir: varsa _2, _3 ... -> hicbir kosulda
 *    mevcut dosyanin ustune yazilmaz (ayni oturumda respawn eden node dahil).
 * Tarih yalnizca bilgi amacli kalir (NTP varsa dogrudur).
 *
 * car_detector ayni mantigin kopyasini tasir (paketler birbirine bagimli degil).
 */
#pragma once

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>

namespace lps {

inline std::string claimRecordPath(const std::string &dir, const std::string &prefix,
                                   int session, const std::string &ext)
{
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);   // varsa sorun degil

    char ts[32];
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
    char ses[16] = "";
    if (session > 0) std::snprintf(ses, sizeof(ses), "_s%04d", session);
    const std::string base = dir + "/" + prefix + ses + "_" + ts;

    for (int k = 1; k < 100; ++k) {
        const std::string p = base + (k == 1 ? "" : "_" + std::to_string(k)) + "." + ext;
        int fd = ::open(p.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0644);
        if (fd >= 0) { ::close(fd); return p; }
        if (errno != EEXIST) return p;   // dizin yazilamiyor vb.: acan taraf hatayi raporlar
    }
    return base + "_p" + std::to_string(::getpid()) + "." + ext;
}

}  // namespace lps
