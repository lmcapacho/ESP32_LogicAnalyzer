#include "i2s_dma_hal_esp32.h"

namespace i2s_dma_hal {

bool init_esp32(const Config &cfg)
{
    (void)cfg;
    return true;
}

void start_esp32() {}

void stop_esp32() {}

} // namespace i2s_dma_hal
