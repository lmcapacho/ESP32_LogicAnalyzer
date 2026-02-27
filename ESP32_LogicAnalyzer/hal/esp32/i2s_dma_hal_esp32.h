#pragma once

#include "../i2s_dma_hal.h"

namespace i2s_dma_hal {

bool init_esp32(const Config &cfg);
void start_esp32();
void stop_esp32();

} // namespace i2s_dma_hal
