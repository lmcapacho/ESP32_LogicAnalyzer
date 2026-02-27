#include "i2s_dma_hal.h"

#if defined(CONFIG_IDF_TARGET_ESP32)
#include "esp32/i2s_dma_hal_esp32.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp32s3/i2s_dma_hal_esp32s3.h"
#endif

namespace i2s_dma_hal {

static bool s_initialized = false;
static void *s_legacy_ctx = nullptr;
static LegacyOps s_legacy_ops = {};

bool init(const Config &cfg)
{
#if defined(CONFIG_IDF_TARGET_ESP32)
    s_initialized = init_esp32(cfg);
    return s_initialized;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    s_initialized = init_esp32s3(cfg);
    if (s_initialized)
    {
        s_legacy_ctx = nullptr;
        s_legacy_ops = legacy_ops_esp32s3();
    }
    return s_initialized;
#else
    (void)cfg;
    s_initialized = false;
    return false;
#endif
}

void start()
{
    if (!s_initialized)
        return;
#if defined(CONFIG_IDF_TARGET_ESP32)
    start_esp32();
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    start_esp32s3();
#endif
}

void stop()
{
    if (!s_initialized)
        return;
#if defined(CONFIG_IDF_TARGET_ESP32)
    stop_esp32();
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    stop_esp32s3();
#endif
}

void bind_legacy_ops(void *ctx, const LegacyOps &ops)
{
    s_legacy_ctx = ctx;
    s_legacy_ops = ops;
}

esp_err_t dma_desc_init(int raw_byte_size)
{
    if (!s_legacy_ops.dma_desc_init)
        return ESP_ERR_NOT_SUPPORTED;
    return s_legacy_ops.dma_desc_init(s_legacy_ctx, raw_byte_size);
}

void i2s_parallel_setup(const i2s_parallel_config_t *cfg)
{
    if (!s_legacy_ops.i2s_parallel_setup)
        return;
    s_legacy_ops.i2s_parallel_setup(s_legacy_ctx, cfg);
}

void start_dma_capture()
{
    if (!s_legacy_ops.start_dma_capture)
        return;
    s_legacy_ops.start_dma_capture(s_legacy_ctx);
}

} // namespace i2s_dma_hal
