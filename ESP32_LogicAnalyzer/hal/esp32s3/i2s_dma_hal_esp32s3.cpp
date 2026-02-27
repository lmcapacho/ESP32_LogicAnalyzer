#include "i2s_dma_hal_esp32s3.h"

namespace i2s_dma_hal {

static esp_err_t dma_desc_init_s3(void *ctx, int raw_byte_size)
{
    (void)ctx;
    (void)raw_byte_size;
    return ESP_ERR_NOT_SUPPORTED;
}

static void i2s_parallel_setup_s3(void *ctx, const i2s_parallel_config_t *cfg)
{
    (void)ctx;
    (void)cfg;
}

static void start_dma_capture_s3(void *ctx)
{
    (void)ctx;
}

bool init_esp32s3(const Config &cfg)
{
    (void)cfg;
    return true;
}

void start_esp32s3() {}

void stop_esp32s3() {}

const LegacyOps &legacy_ops_esp32s3()
{
    static LegacyOps ops = {};
    ops.dma_desc_init = dma_desc_init_s3;
    ops.i2s_parallel_setup = i2s_parallel_setup_s3;
    ops.start_dma_capture = start_dma_capture_s3;
    return ops;
}

} // namespace i2s_dma_hal
