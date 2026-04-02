#include "capture_backend_esp32.h"

namespace capture_backend_esp32 {

bool init(const i2s_dma_hal::Config &cfg)
{
    (void)cfg;
    return true;
}

void start() {}

void stop() {}

} // namespace capture_backend_esp32
