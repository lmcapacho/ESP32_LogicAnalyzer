#pragma once

#if __has_include("soc/lldesc.h")
#include "soc/lldesc.h"
#elif __has_include("esp_private/dma_types.h")
#include "esp_private/dma_types.h"
#elif __has_include("hal/dma_types.h")
#include "hal/dma_types.h"
#else
#include "rom/lldesc.h"
#endif

#if __has_include("soc/i2s_struct.h")
#include "soc/i2s_struct.h"
#elif __has_include("esp32/soc/i2s_struct.h")
#include "esp32/soc/i2s_struct.h"
#elif __has_include("esp32s3/soc/i2s_struct.h")
#include "esp32s3/soc/i2s_struct.h"
#endif

#if __has_include("esp_rom_gpio.h")
#include "esp_rom_gpio.h"
#elif __has_include("esp32s3/rom/gpio.h")
#include "esp32s3/rom/gpio.h"
#elif __has_include("esp32/rom/gpio.h")
#include "esp32/rom/gpio.h"
#endif

#if __has_include("soc/io_mux_reg.h")
#include "soc/io_mux_reg.h"
#elif __has_include("esp32/soc/io_mux_reg.h")
#include "esp32/soc/io_mux_reg.h"
#elif __has_include("esp32s3/soc/io_mux_reg.h")
#include "esp32s3/soc/io_mux_reg.h"
#endif

#if __has_include("soc/gpio_periph.h")
#include "soc/gpio_periph.h"
#elif __has_include("esp32/soc/gpio_periph.h")
#include "esp32/soc/gpio_periph.h"
#elif __has_include("esp32s3/soc/gpio_periph.h")
#include "esp32s3/soc/gpio_periph.h"
#endif
