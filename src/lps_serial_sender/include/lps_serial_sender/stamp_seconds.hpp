/**
 * stamp_seconds.hpp — mesaj zaman damgasini saniyeye cevir, ISTISNASIZ.
 *
 * rclcpp::Time(msg->header.stamp) sec < 0 iken "cannot store a negative time point
 * in rclcpp::Time" firlatir ve yakalanmazsa node'u oldurur. 2026-10-20'de Jetson saati
 * 1970'ten acilip NTP ile 2026'ya atlarken RealSense negatif damgali kare yayinladi:
 * serial_sender, iki cpu_apriltag ve infra2_recorder AYNI SANIYEDE coktu (otopilota
 * paket gitmedi). Damgayi kullanan kod (kare zamanlayici, aralik istatistigi) negatif
 * / sicrayan degeri zaten tolere ediyor; cevrimin kendisi atmamali.
 */
#pragma once

#include <builtin_interfaces/msg/time.hpp>

namespace lps {

inline double stampSeconds(const builtin_interfaces::msg::Time &t)
{
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

}  // namespace lps
