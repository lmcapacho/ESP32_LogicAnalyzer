#include "../../LogicAnalyzer.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
extern LogicAnalyzer esp32la;

void IRAM_ATTR esp32_capture_isr_wrapper(void *arg)
{
    esp32la.esp32_capture_isr(arg);
}
#endif
