#pragma once

#include "../i2s_dma_hal.h"

struct i2s_parallel_config_t;

namespace capture_backend_esp32s3 {

bool init(const i2s_dma_hal::Config &cfg);
void start();
void stop();
bool configure(void *ctx, const i2s_parallel_config_t *cfg, int raw_byte_size);
bool capture(void *ctx, uint32_t timeout_ms);

} // namespace capture_backend_esp32s3
