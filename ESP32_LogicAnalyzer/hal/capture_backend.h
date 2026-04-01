#pragma once

#include "esp_err.h"

struct i2s_parallel_config_t;

namespace capture_backend {

struct Hooks {
    esp_err_t (*allocate_state)(void *ctx, int raw_byte_size);
    void (*configure_bus)(void *ctx, const i2s_parallel_config_t *cfg);
    void (*arm_capture)(void *ctx);
};

struct InitConfig {
    void *ctx;
    int gpio_clk_in;
    int bits;
    const i2s_parallel_config_t *parallel;
    int raw_byte_size;
    const Hooks *hooks;
};

bool init(const InitConfig &cfg);
bool capture(void *ctx, uint32_t timeout_ms);

} // namespace capture_backend
