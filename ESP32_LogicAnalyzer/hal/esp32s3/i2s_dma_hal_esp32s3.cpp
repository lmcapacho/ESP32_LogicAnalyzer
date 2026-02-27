#include "i2s_dma_hal_esp32s3.h"
#include "esp_log.h"

namespace i2s_dma_hal {

namespace {
constexpr const char *TAG = "i2s_dma_hal_s3";
Config s_cfg = {};
bool s_cfg_valid = false;
bool s_warned_not_supported = false;
} // namespace

static esp_err_t dma_desc_init_s3(void *ctx, int raw_byte_size)
{
    (void)ctx;
    if (!s_cfg_valid)
        return ESP_ERR_INVALID_STATE;

    if (raw_byte_size <= 0)
        return ESP_ERR_INVALID_ARG;

    if (!s_warned_not_supported)
    {
        ESP_LOGW(TAG, "ESP32-S3 DMA capture backend not implemented yet (requested %d bytes)", raw_byte_size);
        s_warned_not_supported = true;
    }

    return ESP_ERR_NOT_SUPPORTED;
}

static void i2s_parallel_setup_s3(void *ctx, const i2s_parallel_config_t *cfg)
{
    (void)ctx;
    (void)cfg; // Placeholder until LCD_CAM/I2S S3 setup is ported.
}

static void start_dma_capture_s3(void *ctx)
{
    (void)ctx;
    ESP_LOGD(TAG, "start_dma_capture_s3() ignored: backend pending implementation");
}

bool init_esp32s3(const Config &cfg)
{
    s_cfg = cfg;
    s_cfg_valid = true;
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
