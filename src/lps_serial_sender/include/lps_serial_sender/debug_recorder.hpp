/**
 * debug_recorder.hpp — ucus sonrasi analiz icin overlay'li video kaydi
 *
 * Kameranin gordugunu, AprilTag tespitlerini (koseler + ID) ve optik
 * akis durumunu (ok + quality/tex/gradyan HUD'u) tek videoda birlestirir.
 * Amac: ucustan sonra "tespit nerede/neden zorlandi" sorusunu goruntuyle
 * cevaplamak (motion blur, doku kaybi, tag cikisi, satürasyon...).
 *
 * Tasarim kisitlari:
 *  - Ucus-kritik yollara dokunmaz: push() sadece pointer + kucuk struct
 *    kopyalar (frame klonlanmaz, keepalive shared_ptr ile canli tutulur);
 *    resize/cizim/encode ayri DUSUK oncelikli thread'de.
 *  - Tek slot kuyruk: recorder yetisemezse ara kareler sessizce duser,
 *    uretici asla beklemez.
 *  - ROS bagimsizdir; cagiran taraf State'i doldurur.
 */
#pragma once

#include <opencv2/opencv.hpp>

#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace lps {

class DebugRecorder
{
public:
    struct TagOverlay {
        int         id;
        cv::Point2f corners[4];   // tespit goruntusu piksel koordinatlari
    };

    struct State {
        std::vector<TagOverlay> tags;      // bu karede gorulen TUM tag'ler
        bool     tag_valid = false;        // avgPos.valid (ID 45)
        bool     tag_stale = false;        // timeout'a dusmus bayat veri
        float    px = 0, py = 0, pz = 0;   // avgPos (m)
        float    roll = 0, pitch = 0, yaw = 0;   // derece
        float    dx = 0, dy = 0;           // flow EMA (px/frame, tam cozunurluk)
        uint8_t  quality = 0;
        float    consist = 0, tex = 0;
        uint8_t  grad_p50 = 0, grad_p90 = 0;
        uint32_t t_ms = 0;                 // node start'tan beri ms
    };

    DebugRecorder(const std::string &dir, double fps, double scale)
        : fps_(fps < 1.0 ? 15.0 : fps),
          scale_(scale > 0.05 && scale <= 1.0 ? scale : 0.5)
    {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);   // varsa sorun degil

        char name[64];
        std::time_t t = std::time(nullptr);
        std::tm tm{};
        localtime_r(&t, &tm);
        std::strftime(name, sizeof(name), "lps_%Y%m%d_%H%M%S.avi", &tm);
        path_ = dir + "/" + name;

        thread_ = std::thread(&DebugRecorder::run, this);
    }

    ~DebugRecorder()
    {
        {
            std::lock_guard<std::mutex> lk(m_);
            running_ = false;
        }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
        if (writer_.isOpened()) writer_.release();
    }

    DebugRecorder(const DebugRecorder &) = delete;
    DebugRecorder &operator=(const DebugRecorder &) = delete;

    const std::string &path() const { return path_; }
    bool failed() const { return failed_.load(); }

    /** Kareyi kayda onerir. Kopya yapmaz: frame'in arkasindaki buffer,
     *  keepalive yasadigi surece gecerli olmali (ROS msg shared_ptr'i
     *  verilir). record_fps'e gore seyreltir; recorder mesgulse eski
     *  bekleyen kare ezilir — uretici hicbir kosulda bloklanmaz. */
    void push(const cv::Mat &frame,
              std::shared_ptr<const void> keepalive,
              const State &st)
    {
        if (failed_.load()) return;
        // fps seyreltme (kayan pencere degil, basit esik — debug icin yeter)
        const uint32_t interval_ms = (uint32_t)(1000.0 / fps_);
        if (st.t_ms - last_accept_ms_ < interval_ms && st.t_ms >= last_accept_ms_)
            return;
        last_accept_ms_ = st.t_ms;

        {
            std::lock_guard<std::mutex> lk(m_);
            slot_frame_ = frame;
            slot_keep_  = std::move(keepalive);
            slot_state_ = st;
            pending_    = true;
        }
        cv_.notify_one();
    }

private:
    void run()
    {
        // Dusuk oncelik (nice +10): encode hicbir zaman TX/VPI/apriltag'in
        // onune gecmesin. Basarisizligi onemsiz (best-effort).
        setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), 10);

        while (true) {
            cv::Mat frame;
            std::shared_ptr<const void> keep;
            State st;
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait(lk, [&] { return !running_ || pending_; });
                if (!pending_) break;   // running_ false + is yok → cik
                frame = slot_frame_;
                keep  = std::move(slot_keep_);
                st    = std::move(slot_state_);
                slot_frame_ = cv::Mat();
                pending_ = false;
            }
            writeFrame(frame, st);
            if (failed_.load()) break;
        }
    }

    void writeFrame(const cv::Mat &frame, const State &st)
    {
        if (frame.empty()) return;

        if (!writer_.isOpened()) {
            out_size_ = cv::Size(((int)(frame.cols * scale_)) & ~1,
                                 ((int)(frame.rows * scale_)) & ~1);
            if (!writer_.open(path_,
                              cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                              fps_, out_size_, true)) {
                std::fprintf(stderr,
                    "[DebugRecorder] video acilamadi: %s — kayit KAPALI\n",
                    path_.c_str());
                failed_ = true;
                return;
            }
        }

        cv::resize(frame, small_, out_size_, 0, 0, cv::INTER_AREA);
        if (small_.channels() == 1)
            cv::cvtColor(small_, canvas_, cv::COLOR_GRAY2BGR);
        else
            small_.copyTo(canvas_);

        drawOverlay(canvas_, st,
                    (float)out_size_.width / (float)frame.cols);
        writer_.write(canvas_);
    }

    void drawOverlay(cv::Mat &img, const State &st, float k)
    {
        const cv::Scalar GREEN(80, 220, 80), CYAN(255, 220, 60),
                         RED(60, 60, 230), WHITE(240, 240, 240);

        // Tag cerceveleri + ID
        for (const auto &t : st.tags) {
            cv::Point pts[4];
            for (int i = 0; i < 4; ++i)
                pts[i] = cv::Point((int)(t.corners[i].x * k),
                                   (int)(t.corners[i].y * k));
            const cv::Point *pp = pts;
            int npts = 4;
            cv::polylines(img, &pp, &npts, 1, true, GREEN, 2, cv::LINE_AA);
            cv::putText(img, std::to_string(t.id),
                        pts[0] + cv::Point(3, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, GREEN, 2, cv::LINE_AA);
        }

        // Akis oku (merkezden; hover hareketleri gorunsun diye x15 buyutme)
        {
            cv::Point c(img.cols / 2, img.rows / 2);
            float ax = st.dx * 15.0f * k, ay = st.dy * 15.0f * k;
            float mag = std::sqrt(ax * ax + ay * ay);
            const float maxlen = img.rows * 0.35f;
            if (mag > maxlen) { ax *= maxlen / mag; ay *= maxlen / mag; }
            cv::arrowedLine(img, c,
                            c + cv::Point((int)ax, (int)ay),
                            CYAN, 2, cv::LINE_AA, 0, 0.25);
            cv::circle(img, c, 3, CYAN, -1, cv::LINE_AA);
        }

        // HUD — ust serit (yari saydam koyu zemin, okunurluk icin)
        cv::Rect band(0, 0, img.cols, 44);
        img(band) *= 0.35;

        char l1[160], l2[160];
        const char *tagtxt = st.tag_valid ? (st.tag_stale ? "STALE" : "OK")
                                          : "YOK";
        std::snprintf(l1, sizeof(l1),
            "t=%8.2fs  TAG:%-5s pos %+6.2f %+6.2f %+6.2f m  rpy %+5.0f %+5.0f %+5.0f",
            st.t_ms / 1000.0, tagtxt, st.px, st.py, st.pz,
            st.roll, st.pitch, st.yaw);
        std::snprintf(l2, sizeof(l2),
            "FLOW dx=%+6.2f dy=%+6.2f  Q=%3u  consist=%.2f tex=%.2f g50/90=%u/%u",
            st.dx, st.dy, st.quality, st.consist, st.tex,
            st.grad_p50, st.grad_p90);

        cv::putText(img, l1, {6, 17}, cv::FONT_HERSHEY_SIMPLEX, 0.42,
                    st.tag_valid && !st.tag_stale ? WHITE : RED, 1, cv::LINE_AA);
        cv::putText(img, l2, {6, 36}, cv::FONT_HERSHEY_SIMPLEX, 0.42,
                    WHITE, 1, cv::LINE_AA);
    }

    double      fps_;
    double      scale_;
    std::string path_;

    std::mutex                   m_;
    std::condition_variable      cv_;
    bool                         running_ = true;
    bool                         pending_ = false;
    cv::Mat                      slot_frame_;
    std::shared_ptr<const void>  slot_keep_;
    State                        slot_state_;
    uint32_t                     last_accept_ms_ = 0;

    std::thread          thread_;
    std::atomic<bool>    failed_{false};
    cv::VideoWriter      writer_;
    cv::Size             out_size_;
    cv::Mat              small_, canvas_;   // realloc olmasin diye uye
};

} // namespace lps
