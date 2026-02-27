#pragma once

#include "../i2s_dma_hal.h"

namespace i2s_dma_hal {

bool init_esp32s3(const Config &cfg);
void start_esp32s3();
void stop_esp32s3();
const LegacyOps &legacy_ops_esp32s3();

} // namespace i2s_dma_hal
