#include "i2s_dma_hal_esp32s3.h"

namespace i2s_dma_hal {

bool init_esp32s3(const Config &cfg)
{
    (void)cfg;
    return true;
}

void start_esp32s3() {}

void stop_esp32s3() {}

} // namespace i2s_dma_hal
