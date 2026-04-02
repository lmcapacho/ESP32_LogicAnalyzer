#pragma once

#include "../i2s_dma_hal.h"

namespace capture_backend_esp32 {

bool init(const i2s_dma_hal::Config &cfg);
void start();
void stop();

} // namespace capture_backend_esp32
