#pragma once

#include "i2s_dma_hal.h"

struct i2s_parallel_config_t;

namespace capture_backend {

struct InitConfig {
    void *ctx;
    i2s_dma_hal::Config hal;
    const i2s_parallel_config_t *parallel;
    int raw_byte_size;
    const i2s_dma_hal::LegacyOps *legacy_ops;
};

bool init(const InitConfig &cfg);

} // namespace capture_backend
