#include <gpiod.h>
#include <string>
#include <cstring>
#include <cstdio>
#include "pal/gpio_pin_linux.h"

// v2'de DRIVE_* bayrakları var; yoksa v1 kabul ediyoruz
#if defined(GPIOD_LINE_REQUEST_FLAG_DRIVE_PUSH_PULL)
  #define HRVS_GPIOD_V2 1
#else
  #define HRVS_GPIOD_V2 0
#endif

// İstersen chip/offset'i sabitle (PH.07 sende genelde chip0, offset 50)
// Açmak için aşağıdaki define'ı uncomment et
// #define HRVS_GPIO_FORCE_CHIP_AND_OFFSET 1
// #define HRVS_GPIO_FORCED_CHIP   "/dev/gpiochip0"
// #define HRVS_GPIO_FORCED_OFFSET 50

// v1/v2 uyumlu yardımcılar
static int hrvs_chip_num_lines(struct gpiod_chip* c) {
#if HRVS_GPIOD_V2
    return gpiod_chip_get_num_lines(c);
#else
    return gpiod_chip_num_lines(c);
#endif
}

static const char* hrvs_line_name_of(struct gpiod_line* l) {
#if HRVS_GPIOD_V2
    return gpiod_line_get_name(l);
#else
    return gpiod_line_name(l);
#endif
}


// "PH.07" adına göre chip+offset bul
static int find_line_by_name(const char* name, std::string& chipdev_out, unsigned& offset_out) {
    for (int idx = 0;; ++idx) {
        char dev[32];
        std::snprintf(dev, sizeof(dev), "/dev/gpiochip%d", idx);
        struct gpiod_chip* c = gpiod_chip_open(dev);
        if (!c) break;
        int n = hrvs_chip_num_lines(c);
        for (int i = 0; i < n; ++i) {
            struct gpiod_line* l = gpiod_chip_get_line(c, i);
            if (!l) continue;
            const char* lname = hrvs_line_name_of(l);
            if (lname && std::strcmp(lname, name) == 0) {
                chipdev_out = dev;
                offset_out  = static_cast<unsigned>(i);
                gpiod_chip_close(c);
                return 0;
            }
        }
        gpiod_chip_close(c);
    }
    return -1;
}

bool GPIOPinLinux::configure(unsigned pinId, const char *pioId)
{
    pinIdentifier = pinId;
    configured = false;

    if (pioId && *pioId) std::snprintf(pioIdentifier, sizeof(pioIdentifier), "%s", pioId);
    else                 std::snprintf(pioIdentifier, sizeof(pioIdentifier), "PH.07");

    std::string chipdev;
#ifdef HRVS_GPIO_FORCE_CHIP_AND_OFFSET
    chipdev = HRVS_GPIO_FORCED_CHIP;
    offset  = HRVS_GPIO_FORCED_OFFSET;
#else
    if (find_line_by_name(pioIdentifier, chipdev, offset) != 0) {
        // PH.07 için bilinen fallback (sendeki çıktıya göre)
        chipdev = "/dev/gpiochip0";
        offset  = 50;
        // isimle bulunamazsa bile çoğu cihazda bu doğru; yine de fail etmek istersen return false;
    }
#endif

    chip = gpiod_chip_open(chipdev.c_str());
    if (!chip) return false;

    line = gpiod_chip_get_line(chip, offset);
    if (!line) { gpiod_chip_close(chip); chip=nullptr; return false; }

#if HRVS_GPIOD_V2
    gpiod_line_request_config cfg{};
    cfg.consumer = "hrvs-gpio";
    cfg.request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
    // push-pull + bias disable (gpioset --drive=push-pull ile aynı)
    cfg.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE | GPIOD_LINE_REQUEST_FLAG_DRIVE_PUSH_PULL;
    if (gpiod_line_request(line, &cfg, 0) < 0) {
        gpiod_line_release(line); line=nullptr;
        gpiod_chip_close(chip); chip=nullptr;
        return false;
    }
#else
    // v1: drive flag yok, direkt output request
    if (gpiod_line_request_output(line, "hrvs-gpio", 0) < 0) {
        gpiod_line_release(line); line=nullptr;
        gpiod_chip_close(chip); chip=nullptr;
        return false;
    }
#endif

    configured = true;
    return true;
}

bool GPIOPinLinux::writePinState(State state) const
{
    if (!configured || !line) return false;
    int v = (state == State::ONE) ? 1 : 0;
    return gpiod_line_set_value(line, v) == 0;
}

bool GPIOPinLinux::closePin()
{
    if (line) { gpiod_line_release(line); line=nullptr; }
    if (chip) { gpiod_chip_close(chip);   chip=nullptr; }
    configured = false;
    return true;
}
