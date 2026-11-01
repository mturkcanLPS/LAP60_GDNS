#ifndef GPIO_PIN_LINUX_H
#define GPIO_PIN_LINUX_H

#include "pal_interfaces/i_gpio_pin.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

// forward decl (libgpiod)
struct gpiod_chip;
struct gpiod_line;

class GPIOPinLinux : public hrvs_sdk::IGPIOPin
{
private:
    unsigned pinIdentifier = 0;
    char     pioIdentifier[64] = {0};
    bool     configured = false;

    // libgpiod kaynakları
    gpiod_chip* chip = nullptr;
    gpiod_line* line = nullptr;
    unsigned    offset = 0;

public:
    bool configure(unsigned pinID, const char *pioID);
    bool writePinState(State state) const;
    bool closePin();
};

#endif
