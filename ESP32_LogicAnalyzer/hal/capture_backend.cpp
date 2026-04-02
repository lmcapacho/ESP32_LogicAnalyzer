#include "capture_backend.h"

#include "../LogicAnalyzer.h"
#include "capture_backend_bridge.h"
#include "esp_log.h"
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp32s3/capture_backend_esp32s3.h"
#endif

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp_soc_compat.h"
#endif

namespace capture_backend {

namespace {
constexpr const char *CAPTURE_BACKEND_TAG = "capture_backend";
}

bool init(const InitConfig &cfg)
{
    capture_backend_bridge::Config hal_cfg = {};
    hal_cfg.gpio_clk_in = cfg.gpio_clk_in;
    hal_cfg.bits = cfg.bits;

    if (!capture_backend_bridge::init(hal_cfg))
        return false;

    capture_backend_bridge::bind_legacy_context(cfg.ctx);

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    logic_analyzer_state_t *state = nullptr;
    if (capture_backend_bridge::allocate_dma_state_buffers(&state, cfg.raw_byte_size) != ESP_OK)
        return false;
    capture_backend_bridge::set_logic_state(cfg.ctx, state);
    return capture_backend_esp32s3::configure(cfg.parallel, cfg.raw_byte_size);
#else
    capture_backend_bridge::LegacyOps legacy_ops = {};
    if (cfg.hooks)
    {
        legacy_ops.dma_desc_init = cfg.hooks->allocate_state;
        legacy_ops.i2s_parallel_setup = cfg.hooks->configure_bus;
        legacy_ops.start_dma_capture = cfg.hooks->arm_capture;
    }

    if (cfg.hooks)
        capture_backend_bridge::bind_legacy_ops(cfg.ctx, legacy_ops);

    if (capture_backend_bridge::dma_desc_init(cfg.raw_byte_size) != ESP_OK)
        return false;

    if (cfg.parallel)
        capture_backend_bridge::i2s_parallel_setup(cfg.parallel);
    return true;
#endif
}

Result capture(void *ctx, uint32_t timeout_ms)
{
    logic_analyzer_state_t *state = capture_backend_bridge::get_logic_state(ctx);
    if (!state)
        return Result{false, 0};

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    const Result result = [&]() {
        const auto native = capture_backend_esp32s3::capture(capture_backend_bridge::get_capture_byte_count(ctx), timeout_ms);
        state->dma_done = native.done;
        state->dma_desc_cur = native.completed_desc_count;
        if (native.done)
            capture_backend_esp32s3::copy_to_logic_state(state);
        return Result{native.done, native.completed_desc_count};
    }();
    return result;
#else
    capture_backend_bridge::start_dma_capture();
    capture_backend_bridge::start();
    yield();
    I2S0.conf.rx_start = 1;
    delay(100);

    while (!state->dma_done)
        delay(100);

    capture_backend_bridge::stop();
    return Result{state->dma_done, state->dma_desc_cur};
#endif
}

} // namespace capture_backend
