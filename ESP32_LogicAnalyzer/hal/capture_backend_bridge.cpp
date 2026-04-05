#include "capture_backend_bridge.h"
#include "../LogicAnalyzer.h"
#include <assert.h>
#include <stdlib.h>
#include "esp_heap_caps.h"

#if defined(CONFIG_IDF_TARGET_ESP32)
#include "esp32/capture_backend_esp32.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp32s3/capture_backend_esp32s3.h"
#endif

namespace capture_backend_bridge {

static bool s_initialized = false;
static void *s_legacy_ctx = nullptr;
static LegacyOps s_legacy_ops = {};

bool init(const Config &cfg)
{
#if defined(CONFIG_IDF_TARGET_ESP32)
    s_initialized = capture_backend_esp32::init(cfg);
    return s_initialized;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    s_initialized = capture_backend_esp32s3::init(cfg);
    if (s_initialized)
    {
        s_legacy_ctx = nullptr;
        s_legacy_ops = {};
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
    capture_backend_esp32::start();
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    capture_backend_esp32s3::start();
#endif
}

void stop()
{
    if (!s_initialized)
        return;
#if defined(CONFIG_IDF_TARGET_ESP32)
    capture_backend_esp32::stop();
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    capture_backend_esp32s3::stop();
#endif
}

void bind_legacy_ops(void *ctx, const LegacyOps &ops)
{
    s_legacy_ctx = ctx;
    s_legacy_ops = ops;
}

void bind_legacy_context(void *ctx)
{
    s_legacy_ctx = ctx;
}

esp_err_t dma_desc_init(int raw_byte_size)
{
    if (!s_legacy_ops.dma_desc_init)
        return ESP_ERR_NOT_SUPPORTED;
    return s_legacy_ops.dma_desc_init(s_legacy_ctx, raw_byte_size);
}

void i2s_parallel_setup(const capture_config_t *cfg)
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

logic_analyzer_state_t *get_logic_state(void *ctx)
{
    if (!ctx)
        return nullptr;
    return static_cast<LogicAnalyzer *>(ctx)->s_state;
}

void set_logic_state(void *ctx, logic_analyzer_state_t *state)
{
    if (!ctx)
        return;
    static_cast<LogicAnalyzer *>(ctx)->s_state = state;
}

uint32_t get_capture_byte_count(void *ctx)
{
    if (!ctx)
        return 0;
    return static_cast<LogicAnalyzer *>(ctx)->readCount;
}

esp_err_t allocate_dma_state_buffers(logic_analyzer_state_t **state, int raw_byte_size)
{
    if (!state || raw_byte_size <= 0 || (raw_byte_size % 4) != 0)
        return ESP_ERR_INVALID_ARG;

    logic_analyzer_state_t *new_state = static_cast<logic_analyzer_state_t *>(
        heap_caps_calloc(1, sizeof(logic_analyzer_state_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!new_state)
        return ESP_ERR_NO_MEM;

    assert(raw_byte_size % 4 == 0);

    size_t dma_desc_count = (raw_byte_size + 3999) / 4000;
    size_t buf_size = 4000;
    new_state->dma_buf_width = buf_size;
    new_state->dma_val_per_desc = 2000;
    new_state->dma_sample_per_desc = 1000;
    new_state->dma_desc_count = dma_desc_count;

    new_state->dma_buf = static_cast<capture_dma_elem_t **>(heap_caps_malloc(
        sizeof(capture_dma_elem_t *) * dma_desc_count, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!new_state->dma_buf)
    {
        free(new_state);
        return ESP_ERR_NO_MEM;
    }

    new_state->dma_desc = static_cast<lldesc_t *>(heap_caps_malloc(
        sizeof(lldesc_t) * dma_desc_count, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!new_state->dma_desc)
    {
        free(new_state->dma_buf);
        free(new_state);
        return ESP_ERR_NO_MEM;
    }

    size_t dma_sample_count = 0;
    for (size_t i = 0; i < dma_desc_count; ++i)
    {
        capture_dma_elem_t *buf = static_cast<capture_dma_elem_t *>(heap_caps_malloc(
            buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        if (!buf)
        {
            for (size_t j = 0; j < i; ++j)
                free(new_state->dma_buf[j]);
            free(new_state->dma_desc);
            free(new_state->dma_buf);
            free(new_state);
            return ESP_ERR_NO_MEM;
        }

        new_state->dma_buf[i] = buf;
        lldesc_t *pd = &new_state->dma_desc[i];
        pd->length = buf_size / 2;
        dma_sample_count += buf_size / 2;
        pd->size = buf_size;
        pd->owner = 1;
        pd->sosf = 1;
        pd->buf = reinterpret_cast<uint8_t *>(buf);
        pd->offset = 0;
        pd->empty = 0;
        pd->eof = 0;
        pd->qe.stqe_next = &new_state->dma_desc[(i + 1) % dma_desc_count];
        if (i + 1 == dma_desc_count)
        {
            pd->eof = 1;
            pd->qe.stqe_next = 0x0;
        }
    }

    new_state->dma_done = false;
    new_state->dma_sample_count = dma_sample_count;
    *state = new_state;
    return ESP_OK;
}

} // namespace capture_backend_bridge
