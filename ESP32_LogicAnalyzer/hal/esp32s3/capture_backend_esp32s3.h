#pragma once

#include "../i2s_dma_hal.h"
#include <stddef.h>

struct i2s_parallel_config_t;

namespace capture_backend_esp32s3 {

struct Result {
    bool done;
    size_t completed_desc_count;
};

struct DebugSnapshot {
    bool dma_ready;
    uint32_t eof_count;
    uintptr_t last_eof_desc;
    uint32_t last_first_word;
};

bool init(const i2s_dma_hal::Config &cfg);
void start();
void stop();
bool configure(const i2s_parallel_config_t *cfg, int raw_byte_size);
Result capture(size_t capture_bytes, uint32_t timeout_ms);
void copy_to_logic_state(logic_analyzer_state_t *state);
DebugSnapshot debug_snapshot();

} // namespace capture_backend_esp32s3
