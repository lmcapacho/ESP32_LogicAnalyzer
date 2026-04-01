#include "capture_backend.h"

#include "../LogicAnalyzer.h"
#include "i2s_dma_hal.h"
#include "esp_log.h"
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp32s3/capture_backend_esp32s3.h"
#endif

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#if __has_include("soc/i2s_struct.h")
#include "soc/i2s_struct.h"
#elif __has_include("esp32/soc/i2s_struct.h")
#include "esp32/soc/i2s_struct.h"
#elif __has_include("esp32s3/soc/i2s_struct.h")
#include "esp32s3/soc/i2s_struct.h"
#endif
#endif

namespace capture_backend {

namespace {
constexpr const char *CAPTURE_BACKEND_TAG = "capture_backend";
}

bool init(const InitConfig &cfg)
{
    i2s_dma_hal::Config hal_cfg = {};
    hal_cfg.gpio_clk_in = cfg.gpio_clk_in;
    hal_cfg.bits = cfg.bits;

    if (!i2s_dma_hal::init(hal_cfg))
        return false;

    i2s_dma_hal::bind_legacy_context(cfg.ctx);

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    logic_analyzer_state_t *state = nullptr;
    if (i2s_dma_hal::allocate_dma_state_buffers(&state, cfg.raw_byte_size) != ESP_OK)
        return false;
    i2s_dma_hal::set_logic_state(cfg.ctx, state);
    return capture_backend_esp32s3::configure(cfg.parallel, cfg.raw_byte_size);
#else
    i2s_dma_hal::LegacyOps legacy_ops = {};
    if (cfg.hooks)
    {
        legacy_ops.dma_desc_init = cfg.hooks->allocate_state;
        legacy_ops.i2s_parallel_setup = cfg.hooks->configure_bus;
        legacy_ops.start_dma_capture = cfg.hooks->arm_capture;
    }

    if (cfg.hooks)
        i2s_dma_hal::bind_legacy_ops(cfg.ctx, legacy_ops);

    if (i2s_dma_hal::dma_desc_init(cfg.raw_byte_size) != ESP_OK)
        return false;

    if (cfg.parallel)
        i2s_dma_hal::i2s_parallel_setup(cfg.parallel);
    return true;
#endif
}

Result capture(void *ctx, uint32_t timeout_ms)
{
    logic_analyzer_state_t *state = i2s_dma_hal::get_logic_state(ctx);
    if (!state)
        return Result{false, 0};

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    const Result result = [&]() {
        const auto native = capture_backend_esp32s3::capture(i2s_dma_hal::get_capture_byte_count(ctx), timeout_ms);
        state->dma_done = native.done;
        state->dma_desc_cur = native.completed_desc_count;
        if (native.done)
            capture_backend_esp32s3::copy_to_logic_state(state);
        return Result{native.done, native.completed_desc_count};
    }();
    return result;
#else
    i2s_dma_hal::start_dma_capture();
    i2s_dma_hal::start();
    yield();
    I2S0.conf.rx_start = 1;
    delay(100);

    while (!state->dma_done)
        delay(100);

    i2s_dma_hal::stop();
    return Result{state->dma_done, state->dma_desc_cur};
#endif
}

} // namespace capture_backend
