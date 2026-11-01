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

#include "lps_serial_sender/record_naming.hpp"

namespace lps {

/** Kayit seyreltici: girdi ne hizda gelirse gelsin ORTALAMADA tam hedef fps.
 *
 *  Eski kural 'son kabulden beri >= aralik' idi. Aralik (66 ms) iki kare
 *  periyoduna esit/yakin oldugunda jitter her ikinci karede esigi kacirtir:
 *  21-25 Hz girdide ~10.5-12 fps kaydediyordu — video 15 fps oynatildigi icin
 *  gercek zamandan ~1.4x HIZLI akiyordu. Burada sabit takvim var: kabul
 *  aninda sonraki hedef 'kabul anina' degil 'onceki hedefe' aralik eklenerek
 *  kurulur, kacan pay sonraki karelerde telafi edilir. Uzun bosluktan (girdi
 *  hedeften yavas / duraklama) sonra takvim yeniden kurulur ki birikmis borc
 *  arka arkaya kare kabuluyle patlamasin. uint32 ms tasmasina karsi guvenli. */
class FrameThinner
{
public:
    explicit FrameThinner(double fps) : iv_((uint32_t)(1000.0 / fps)) {}
    bool accept(uint32_t t_ms)
    {
        if (have_ && (int32_t)(t_ms - next_) < 0) return false;   // zamani gelmedi
        if (!have_ || (int32_t)(t_ms - next_) > (int32_t)iv_)
            next_ = t_ms + iv_;          // ilk kare / uzun bosluk: takvimi kur
        else
            next_ += iv_;                // takvimde kal (kayan pay telafi edilir)
        have_ = true;
        return true;
    }
private:
    uint32_t iv_;
    uint32_t next_ = 0;
    bool     have_ = false;
};

class DebugRecorder
{
public:
    struct TagOverlay {
        int         id;
        cv::Point2f corners[4];   // tespit goruntusu piksel koordinatlari
    };

    /** IR LED dik ucgeni: pts[0] = dik acı kosesi, pts[1..2] = bacak uclari
     *  (hipotenus uclari). mid = hipotenus orta noktasi = HUNI MERKEZI,
     *  pozisyon buradan cikar. */
    struct IrOverlay {
        bool        valid = false;
        bool        no_alt = false;   // radar yok -> capraz kontrol atlandi (IR calisir)
        int         n_blobs = 0;      // eslesme olmasa da kac parlak nokta goruldu
        float       fit_err = 0;      // kenar basina ortalama goreli hata
        int         min_area = 0;     // eslesen en kucuk blob alani (menzil sinir)
        cv::Point2f pts[3];
        cv::Point2f mid;              // hipotenus orta noktasi (huni merkezi)
        float       x = 0, y = 0;     // pakete giden NORMALIZE yon (X/Z, Y/Z —
                                      // BOYUTSUZ; metreye otopilot cevirir)
        float       z = 0;            // ucgenden olculen OPTIK menzil; pakete
                                      // GITMIYOR — dogrulama + HUD'da metrik
                                      // karsiligi gostermek icin
        float       z_radar = 0;      // ayni andaki HRVS altitude — karsilastirma
        // Esikten gecen TUM parlak noktalar (<=16). Ucgen eslesmese de cizilir:
        // esik ayarinda 'nerede/neyi yakaladi' sorusu ancak boyle gorulur.
        std::vector<cv::Point2f> blobs;
    };

    struct State {
        std::vector<TagOverlay> tags;      // bu karede gorulen TUM tag'ler
        IrOverlay ir;                      // IR LED ucgeni (varsa)
        bool     tag_valid = false;        // avgPos.valid (ID 45)
        bool     tag_stale = false;        // timeout'a dusmus bayat veri
        float    px = 0, py = 0, pz = 0;   // avgPos (m)
        float    roll = 0, pitch = 0, yaw = 0;   // derece
        float    dx = 0, dy = 0;           // flow EMA (px/frame, tam cozunurluk)
        uint8_t  quality = 0;
        float    consist = 0, tex = 0;
        uint8_t  grad_p50 = 0, grad_p90 = 0;
        uint32_t t_ms = 0;                 // node start'tan beri ms
        // Kamera karesinin ROS zaman damgasi (s). infra1/infra2 donanimda
        // senkron -> iki videoyu kare kare eslestirmek icin ORTAK saat.
        // (t_ms node'a gore, iki node farkli anda acildigi icin kiyaslanamaz.)
        double   stamp_s = 0.0;
        std::string label;                 // alt seritte: hangi kamera / amac
        bool     ir_enabled = true;        // false: bu kamerada IR hesaplanmiyor
    };

    /** name_prefix: dosya adi oneki, kamera basina farkli (lps_infra1 / lps_infra2).
     *  session: launch'in oturum numarasi -> <prefix>_sNNNN_YYYYmmdd_HHMMSS.avi;
     *  ad atomik sahiplenilir, mevcut dosyanin ustune ASLA yazilmaz
     *  (bkz. record_naming.hpp). */
    DebugRecorder(const std::string &dir, double fps, double scale,
                  const std::string &name_prefix = "lps", int session = 0)
        : fps_(fps < 1.0 ? 15.0 : fps),
          scale_(scale > 0.05 && scale <= 1.0 ? scale : 0.5),
          thinner_(fps_)
    {
        path_ = claimRecordPath(dir, name_prefix, session, "avi");

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
        // Ortalamada tam record_fps (bkz. FrameThinner) — video gercek hizda akar
        if (!thinner_.accept(st.t_ms)) return;

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
                         RED(60, 60, 230), WHITE(240, 240, 240),
                         MAGENTA(230, 80, 230), YELLOW(60, 230, 230);

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

        // Esikten gecen tum parlak noktalar — ucgen eslesmese DE cizilir ki
        // sahada esik ayarlarken neyin yakalandigi gorunsun (kirmizi kucuk daire).
        for (const auto &b : st.ir.blobs)
            cv::circle(img, cv::Point((int)(b.x * k), (int)(b.y * k)),
                       4, RED, 1, cv::LINE_AA);

        // IR LED dik ucgeni: her LED daire icinde, aralarina kenar cizgileri.
        // Dik acı kosesi (pts[0]) sari 'R'. Hipotenus orta noktasi = HUNI
        // MERKEZI, konum oradan cikiyor -> nisangah ile isaretli, dik acıdan
        // ortaya ortay cizgisi (|RM| = hipotenus/2) cizilir.
        if (st.ir.valid) {
            cv::Point p[3];
            for (int i = 0; i < 3; ++i)
                p[i] = cv::Point((int)(st.ir.pts[i].x * k),
                                 (int)(st.ir.pts[i].y * k));
            const cv::Point m((int)(st.ir.mid.x * k), (int)(st.ir.mid.y * k));

            for (int i = 0; i < 3; ++i)
                cv::line(img, p[i], p[(i + 1) % 3], MAGENTA, 1, cv::LINE_AA);
            cv::line(img, p[0], m, YELLOW, 1, cv::LINE_AA);   // ortay
            for (int i = 0; i < 3; ++i) {
                const cv::Scalar c = (i == 0) ? YELLOW : MAGENTA;
                cv::circle(img, p[i], 9, c, 2, cv::LINE_AA);
            }
            cv::putText(img, "R", p[0] + cv::Point(11, -8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, YELLOW, 1, cv::LINE_AA);

            // Huni merkezi — nisangah (konum BU noktadan olculuyor)
            cv::circle(img, m, 11, CYAN, 2, cv::LINE_AA);
            cv::line(img, m + cv::Point(-16, 0), m + cv::Point(16, 0), CYAN, 1, cv::LINE_AA);
            cv::line(img, m + cv::Point(0, -16), m + cv::Point(0, 16), CYAN, 1, cv::LINE_AA);
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
        cv::Rect band(0, 0, img.cols, 63);
        img(band) *= 0.35;

        char l1[160], l2[160], l3[160];
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
        // IR basarisizliginin SEBEBI ayirt edilir: radar yoksa olcek hic yok,
        // 'ucgen eslesmedi' teshisi yaniltici olurdu.
        // Gonderilen NORMALIZE yon; parantezde Zopt ile metrik karsiligi
        // (sahada gozle kontrol icin — o carpimi otopilot kendi Z'siyle yapar).
        if (!st.ir_enabled)
            std::snprintf(l3, sizeof(l3), "IR: bu kamerada hesaplanmiyor");
        else if (st.ir.valid)
            std::snprintf(l3, sizeof(l3),
                "IR:OK  yon %+.4f %+.4f  [Zopt %.2f -> %+5.2f %+5.2f m]%s  "
                "blob=%d amin=%dpx err=%.2f",
                st.ir.x, st.ir.y, st.ir.z,
                st.ir.x * st.ir.z, st.ir.y * st.ir.z,
                st.ir.no_alt ? " (radar yok)" : "",
                st.ir.n_blobs, st.ir.min_area, st.ir.fit_err);
        else
            std::snprintf(l3, sizeof(l3),
                "IR:YOK  ucgen eslesmedi                blob=%d err=%.2f",
                st.ir.n_blobs, st.ir.fit_err);

        cv::putText(img, l1, {6, 17}, cv::FONT_HERSHEY_SIMPLEX, 0.42,
                    st.tag_valid && !st.tag_stale ? WHITE : RED, 1, cv::LINE_AA);
        cv::putText(img, l2, {6, 36}, cv::FONT_HERSHEY_SIMPLEX, 0.42,
                    WHITE, 1, cv::LINE_AA);
        cv::putText(img, l3, {6, 55}, cv::FONT_HERSHEY_SIMPLEX, 0.42,
                    !st.ir_enabled ? WHITE : (st.ir.valid ? YELLOW : RED),
                    1, cv::LINE_AA);

        // Alt serit: kamera etiketi + karenin ROS saati (HH:MM:SS.mmm). Iki
        // kameranin videosunu yan yana izlerken ayni ani bulmak icin — kareler
        // donanim senkronlu, damgalar ayni olur.
        if (!st.label.empty() || st.stamp_s > 0.0) {
            char clk[32] = "";
            if (st.stamp_s > 0.0) {
                std::time_t sec = (std::time_t)st.stamp_s;
                int ms = (int)((st.stamp_s - (double)sec) * 1000.0);
                std::tm tm{};
                localtime_r(&sec, &tm);
                std::snprintf(clk, sizeof(clk), "%02d:%02d:%02d.%03d",
                              tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
            }
            char l4[160];
            std::snprintf(l4, sizeof(l4), "%s   %s", st.label.c_str(), clk);
            cv::Rect bottom(0, img.rows - 22, img.cols, 22);
            img(bottom) *= 0.35;
            cv::putText(img, l4, {6, img.rows - 7}, cv::FONT_HERSHEY_SIMPLEX,
                        0.45, YELLOW, 1, cv::LINE_AA);
        }
    }

    double      fps_;
    double      scale_;
    FrameThinner thinner_;   // sadece push() (uretici thread) kullanir
    std::string path_;

    std::mutex                   m_;
    std::condition_variable      cv_;
    bool                         running_ = true;
    bool                         pending_ = false;
    cv::Mat                      slot_frame_;
    std::shared_ptr<const void>  slot_keep_;
    State                        slot_state_;

    std::thread          thread_;
    std::atomic<bool>    failed_{false};
    cv::VideoWriter      writer_;
    cv::Size             out_size_;
    cv::Mat              small_, canvas_;   // realloc olmasin diye uye
};

} // namespace lps
