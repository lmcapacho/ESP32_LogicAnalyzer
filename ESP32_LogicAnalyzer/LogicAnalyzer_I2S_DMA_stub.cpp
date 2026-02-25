#include "LogicAnalyzer.h"

#if !defined(CONFIG_IDF_TARGET_ESP32)

void LogicAnalyzer::start_dma_capture(void) {}

void LogicAnalyzer::i2s_isr(void *arg)
{
    (void)arg;
}

void LogicAnalyzer::dma_desc_deinit() {}

esp_err_t LogicAnalyzer::dma_desc_init(int raw_byte_size)
{
    (void)raw_byte_size;
    return ESP_ERR_NOT_SUPPORTED;
}

void LogicAnalyzer::i2s_conf_reset() {}

void LogicAnalyzer::i2s_parallel_setup(const i2s_parallel_config_t *cfg)
{
    (void)cfg;
}

void LogicAnalyzer::dma_serializer(dma_elem_t *dma_buffer)
{
    (void)dma_buffer;
}

void LogicAnalyzer::dmabuff_compresser_ch1(uint8_t *dma_buffer)
{
    (void)dma_buffer;
}

void LogicAnalyzer::dmabuff_compresser_ch2(uint8_t *dma_buffer)
{
    (void)dma_buffer;
}

int calc_needed_dma_descs_for(i2s_parallel_buffer_desc_t *desc)
{
    (void)desc;
    return 0;
}

uint16_t buff_process_trigger_1(uint16_t *buff, int size, bool printit)
{
    (void)buff;
    (void)size;
    (void)printit;
    return 0;
}

uint16_t buff_process_trigger_0(uint16_t *buff, int size, bool printit)
{
    (void)buff;
    (void)size;
    (void)printit;
    return 0;
}

#endif
