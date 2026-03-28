#include "i2s_dma_hal_esp32s3.h"

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "../../LogicAnalyzer.h"
#include "driver/periph_ctrl.h"
#include "esp_log.h"
#include "soc/gpio_sig_map.h"
#include "soc/lcd_cam_struct.h"
#if __has_include("esp32s3/rom/gpio.h")
#include "esp32s3/rom/gpio.h"
#elif __has_include("esp_rom_gpio.h")
#include "esp_rom_gpio.h"
#endif
#if __has_include("soc/io_mux_reg.h")
#include "soc/io_mux_reg.h"
#elif __has_include("esp32s3/soc/io_mux_reg.h")
#include "esp32s3/soc/io_mux_reg.h"
#endif
#if __has_include("soc/gpio_periph.h")
#include "soc/gpio_periph.h"
#elif __has_include("esp32s3/soc/gpio_periph.h")
#include "esp32s3/soc/gpio_periph.h"
#endif

namespace i2s_dma_hal {

namespace {
constexpr const char *S3_HAL_TAG = "i2s_dma_hal_s3";
Config s_cfg = {};
bool s_cfg_valid = false;
bool s_warned_not_supported = false;

void gpio_setup_in_s3(int gpio, int sig)
{
    if (gpio < 0)
        return;
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction((gpio_num_t)gpio, (gpio_mode_t)GPIO_MODE_DEF_INPUT);
    gpio_matrix_in(gpio, sig, false);
}
} // namespace

static esp_err_t dma_desc_init_s3(void *ctx, int raw_byte_size)
{
    (void)ctx;
    if (!s_cfg_valid)
        return ESP_ERR_INVALID_STATE;

    if (raw_byte_size <= 0)
        return ESP_ERR_INVALID_ARG;

    if (!s_warned_not_supported)
    {
        ESP_LOGW(S3_HAL_TAG, "ESP32-S3 DMA capture backend not implemented yet (requested %d bytes)", raw_byte_size);
        s_warned_not_supported = true;
    }

    return ESP_ERR_NOT_SUPPORTED;
}

static void i2s_parallel_setup_s3(void *ctx, const i2s_parallel_config_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return;

    ESP_LOGI(S3_HAL_TAG, "Setting up ESP32-S3 LCD_CAM parallel input backend");

    periph_module_enable(PERIPH_LCD_CAM_MODULE);

    LCD_CAM.cam_ctrl.val = 0;
    LCD_CAM.cam_ctrl1.val = 0;
    LCD_CAM.cam_rgb_yuv.val = 0;
    LCD_CAM.lc_dma_int_ena.val = 0;
    LCD_CAM.lc_dma_int_clr.val = 0xFFFFFFFF;

    LCD_CAM.cam_ctrl1.cam_reset = 1;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 1;

    LCD_CAM.cam_ctrl.cam_clk_sel = 2;
    LCD_CAM.cam_ctrl.cam_clkm_div_num = 4;
    LCD_CAM.cam_ctrl.cam_clkm_div_a = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_b = 0;
    LCD_CAM.cam_ctrl.cam_stop_en = 0;
    LCD_CAM.cam_ctrl.cam_vs_eof_en = 0;
    LCD_CAM.cam_ctrl.cam_update = 1;

    LCD_CAM.cam_ctrl1.cam_clk_inv = 0;
    LCD_CAM.cam_ctrl1.cam_2byte_en = (cfg->bits > 8);
    LCD_CAM.cam_ctrl1.cam_vh_de_mode_en = 1;
    LCD_CAM.cam_ctrl1.cam_start = 0;

    gpio_setup_in_s3(cfg->gpio_bus[0], CAM_DATA_IN0_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[1], CAM_DATA_IN1_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[2], CAM_DATA_IN2_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[3], CAM_DATA_IN3_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[4], CAM_DATA_IN4_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[5], CAM_DATA_IN5_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[6], CAM_DATA_IN6_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[7], CAM_DATA_IN7_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[8], CAM_DATA_IN8_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[9], CAM_DATA_IN9_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[10], CAM_DATA_IN10_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[11], CAM_DATA_IN11_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[12], CAM_DATA_IN12_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[13], CAM_DATA_IN13_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[14], CAM_DATA_IN14_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[15], CAM_DATA_IN15_IDX);
    gpio_setup_in_s3(cfg->gpio_clk_in, CAM_PCLK_IDX);

    gpio_matrix_in(0x38, CAM_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, CAM_H_SYNC_IDX, false);
    gpio_matrix_in(0x38, CAM_H_ENABLE_IDX, false);
}

static void start_dma_capture_s3(void *ctx)
{
    (void)ctx;
    ESP_LOGD(S3_HAL_TAG, "start_dma_capture_s3() ignored: backend pending implementation");
}

bool init_esp32s3(const Config &cfg)
{
    s_cfg = cfg;
    s_cfg_valid = true;
    return true;
}

void start_esp32s3() {}

void stop_esp32s3() {}

const LegacyOps &legacy_ops_esp32s3()
{
    static LegacyOps ops = {};
    ops.dma_desc_init = dma_desc_init_s3;
    ops.i2s_parallel_setup = i2s_parallel_setup_s3;
    ops.start_dma_capture = start_dma_capture_s3;
    return ops;
}

} // namespace i2s_dma_hal

#endif
