#pragma once

struct i2s_parallel_config_t;

#include "esp_err.h"

namespace i2s_dma_hal {

struct Config {
    int gpio_clk_in;
    int bits;
};

// Target-specific backend should implement these APIs.
bool init(const Config &cfg);
void start();
void stop();

struct LegacyOps {
    esp_err_t (*dma_desc_init)(void *ctx, int raw_byte_size);
    void (*i2s_parallel_setup)(void *ctx, const i2s_parallel_config_t *cfg);
    void (*start_dma_capture)(void *ctx);
};

void bind_legacy_ops(void *ctx, const LegacyOps &ops);
esp_err_t dma_desc_init(int raw_byte_size);
void i2s_parallel_setup(const i2s_parallel_config_t *cfg);
void start_dma_capture();

} // namespace i2s_dma_hal
