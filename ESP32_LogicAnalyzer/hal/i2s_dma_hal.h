#pragma once

namespace i2s_dma_hal {

struct Config {
    int gpio_clk_in;
    int bits;
};

// Target-specific backend should implement these APIs.
bool init(const Config &cfg);
void start();
void stop();

} // namespace i2s_dma_hal
