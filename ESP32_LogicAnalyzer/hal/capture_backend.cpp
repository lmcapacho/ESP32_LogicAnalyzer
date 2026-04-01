#include "capture_backend.h"

namespace capture_backend {

bool init(const InitConfig &cfg)
{
    if (!i2s_dma_hal::init(cfg.hal))
        return false;

    i2s_dma_hal::bind_legacy_context(cfg.ctx);
    if (cfg.legacy_ops)
        i2s_dma_hal::bind_legacy_ops(cfg.ctx, *cfg.legacy_ops);

    if (i2s_dma_hal::dma_desc_init(cfg.raw_byte_size) != ESP_OK)
        return false;

    if (cfg.parallel)
        i2s_dma_hal::i2s_parallel_setup(cfg.parallel);
    return true;
}

} // namespace capture_backend
