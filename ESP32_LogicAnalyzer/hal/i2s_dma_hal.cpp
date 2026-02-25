#include "i2s_dma_hal.h"

#if defined(CONFIG_IDF_TARGET_ESP32)
#include "esp32/i2s_dma_hal_esp32.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp32s3/i2s_dma_hal_esp32s3.h"
#endif

namespace i2s_dma_hal {

static bool s_initialized = false;

bool init(const Config &cfg)
{
#if defined(CONFIG_IDF_TARGET_ESP32)
    s_initialized = init_esp32(cfg);
    return s_initialized;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    s_initialized = init_esp32s3(cfg);
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
}

void stop()
{
    if (!s_initialized)
        return;
}

} // namespace i2s_dma_hal
