#include "i2s_dma_hal_esp32s3.h"

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "../../LogicAnalyzer.h"
#include "driver/periph_ctrl.h"
#include "esp_heap_caps.h"
#include "esp_private/gdma.h"
#include "esp_private/gpio.h"
#include "esp_log.h"
#include "hal/dma_types.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/lcd_cam_struct.h"
#if __has_include("esp_rom_gpio.h")
#include "esp_rom_gpio.h"
#elif __has_include("esp32s3/rom/gpio.h")
#include "esp32s3/rom/gpio.h"
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
struct S3BackendState {
    gdma_channel_handle_t dma_chan = nullptr;
    dma_descriptor_t *dma_desc = nullptr;
    size_t dma_desc_count = 0;
    size_t dma_desc_bytes = 0;
    logic_analyzer_state_t *active_state = nullptr;
    bool dma_ready = false;
    volatile uint32_t eof_count = 0;
    volatile intptr_t last_eof_desc = 0;
    volatile uint32_t last_first_word = 0;
};

S3BackendState s_s3 = {};

inline void cam_reset_and_fifo()
{
    LCD_CAM.cam_ctrl1.cam_reset = 1;
    LCD_CAM.cam_ctrl1.cam_reset = 0;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 1;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 0;
}

inline void cam_stop_streaming()
{
    LCD_CAM.cam_ctrl1.cam_start = 0;
}

inline void cam_start_streaming()
{
    cam_reset_and_fifo();
    LCD_CAM.cam_ctrl1.cam_start = 1;
}

inline void route_cam_control_idle()
{
    esp_rom_gpio_connect_in_signal(GPIO_FUNC_IN_LOW, CAM_V_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_FUNC_IN_HIGH, CAM_H_ENABLE_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_FUNC_IN_HIGH, CAM_H_SYNC_IDX, false);
}

inline void route_cam_control_active()
{
    esp_rom_gpio_connect_in_signal(GPIO_FUNC_IN_HIGH, CAM_V_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_FUNC_IN_HIGH, CAM_H_ENABLE_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_FUNC_IN_HIGH, CAM_H_SYNC_IDX, false);
}

void configure_dma_descriptors(logic_analyzer_state_t *state)
{
    if (!s_s3.dma_desc || !state)
        return;

    for (size_t i = 0; i < s_s3.dma_desc_count; ++i)
    {
        dma_descriptor_t &desc = s_s3.dma_desc[i];
        desc.dw0.size = state->dma_desc[i].size;
        desc.dw0.length = 0;
        desc.dw0.err_eof = 0;
        desc.dw0.suc_eof = (i + 1 == s_s3.dma_desc_count);
        desc.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        desc.buffer = reinterpret_cast<uint8_t *>(state->dma_buf[i]);
        desc.next = (i + 1 < s_s3.dma_desc_count) ? &s_s3.dma_desc[i + 1] : nullptr;
    }
}

bool IRAM_ATTR on_gdma_recv_eof_s3(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data)
{
    (void)dma_chan;
    (void)user_data;
    logic_analyzer_state_t *state = s_s3.active_state;
    if (!state)
        return false;

    intptr_t eof_addr = event_data ? event_data->rx_eof_desc_addr : 0;
    s_s3.eof_count++;
    s_s3.last_eof_desc = eof_addr;
    if (s_s3.dma_desc &&
        eof_addr >= reinterpret_cast<intptr_t>(&s_s3.dma_desc[0]) &&
        eof_addr < reinterpret_cast<intptr_t>(&s_s3.dma_desc[s_s3.dma_desc_count]))
    {
        dma_descriptor_t *eof_desc = reinterpret_cast<dma_descriptor_t *>(eof_addr);
        state->dma_desc_cur = static_cast<size_t>(eof_desc - s_s3.dma_desc) + 1;
    }
    else
    {
        state->dma_desc_cur = state->dma_desc_count;
    }

    if (state->dma_buf && state->dma_buf[0])
        s_s3.last_first_word = reinterpret_cast<uint32_t *>(state->dma_buf[0])[0];

    cam_stop_streaming();
    state->dma_done = true;
    return false;
}

void gpio_setup_in_s3(int gpio, int sig)
{
    if (gpio < 0)
        return;
    esp_rom_gpio_pad_select_gpio(gpio);
    gpio_set_direction((gpio_num_t)gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)gpio, GPIO_FLOATING);
    esp_rom_gpio_connect_in_signal(gpio, sig, false);
}
} // namespace

static esp_err_t dma_desc_init_s3(void *ctx, int raw_byte_size)
{
    if (!s_cfg_valid)
        return ESP_ERR_INVALID_STATE;

    if (raw_byte_size <= 0)
        return ESP_ERR_INVALID_ARG;

    logic_analyzer_state_t *state = nullptr;
    esp_err_t err = allocate_dma_state_buffers(&state, raw_byte_size);
    if (err != ESP_OK)
        return err;

    if (s_s3.dma_desc)
    {
        heap_caps_free(s_s3.dma_desc);
        s_s3.dma_desc = nullptr;
        s_s3.dma_desc_count = 0;
        s_s3.dma_desc_bytes = 0;
    }

    s_s3.dma_desc_count = state->dma_desc_count;
    s_s3.dma_desc_bytes = s_s3.dma_desc_count * sizeof(dma_descriptor_t);
    s_s3.dma_desc = static_cast<dma_descriptor_t *>(
        heap_caps_calloc(s_s3.dma_desc_count, sizeof(dma_descriptor_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!s_s3.dma_desc)
        return ESP_ERR_NO_MEM;

    configure_dma_descriptors(state);

    set_logic_state(ctx, state);
    return ESP_OK;
}

static void i2s_parallel_setup_s3(void *ctx, const i2s_parallel_config_t *cfg)
{
    (void)ctx;
    if (!cfg)
        return;

    periph_module_enable(PERIPH_LCD_CAM_MODULE);

    if (!s_s3.dma_chan)
    {
        gdma_channel_alloc_config_t dma_config = {};
        dma_config.direction = GDMA_CHANNEL_DIRECTION_RX;
        if (gdma_new_channel(&dma_config, &s_s3.dma_chan) != ESP_OK)
        {
            ESP_LOGE(S3_HAL_TAG, "Failed to allocate GDMA RX channel for ESP32-S3");
            s_s3.dma_chan = nullptr;
            s_s3.dma_ready = false;
            return;
        }

        gdma_transfer_ability_t ability = {};
        ability.sram_trans_align = 4;
        gdma_set_transfer_ability(s_s3.dma_chan, &ability);

        gdma_strategy_config_t strategy = {};
        strategy.owner_check = true;
        strategy.auto_update_desc = false;
        gdma_apply_strategy(s_s3.dma_chan, &strategy);

        gdma_rx_event_callbacks_t cbs = {};
        cbs.on_recv_eof = on_gdma_recv_eof_s3;
        if (gdma_register_rx_event_callbacks(s_s3.dma_chan, &cbs, ctx) != ESP_OK ||
            gdma_connect(s_s3.dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_CAM, 0)) != ESP_OK)
        {
            ESP_LOGE(S3_HAL_TAG, "Failed to configure GDMA RX channel for LCD_CAM");
            gdma_del_channel(s_s3.dma_chan);
            s_s3.dma_chan = nullptr;
            s_s3.dma_ready = false;
            return;
        }
    }

    LCD_CAM.cam_ctrl.val = 0;
    LCD_CAM.cam_ctrl1.val = 0;
    LCD_CAM.cam_rgb_yuv.val = 0;
    LCD_CAM.lc_dma_int_ena.val = 0;
    LCD_CAM.lc_dma_int_clr.val = 0xFFFFFFFF;

    cam_reset_and_fifo();

    LCD_CAM.cam_ctrl.cam_clk_sel = 2;
    LCD_CAM.cam_ctrl.cam_clkm_div_num = 4;
    LCD_CAM.cam_ctrl.cam_clkm_div_a = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_b = 0;
    LCD_CAM.cam_ctrl.cam_stop_en = 0;
    LCD_CAM.cam_ctrl.cam_vs_eof_en = 0;
    LCD_CAM.cam_ctrl.cam_update = 1;

    LCD_CAM.cam_ctrl1.cam_clk_inv = 0;
    LCD_CAM.cam_ctrl1.cam_2byte_en = 0;
    LCD_CAM.cam_ctrl1.cam_de_inv = 0;
    LCD_CAM.cam_ctrl1.cam_hsync_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vsync_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vh_de_mode_en = 1;
    LCD_CAM.cam_ctrl1.cam_start = 0;
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = 0;

    gpio_setup_in_s3(cfg->gpio_bus[0], CAM_DATA_IN0_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[1], CAM_DATA_IN1_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[2], CAM_DATA_IN2_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[3], CAM_DATA_IN3_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[4], CAM_DATA_IN4_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[5], CAM_DATA_IN5_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[6], CAM_DATA_IN6_IDX);
    gpio_setup_in_s3(cfg->gpio_bus[7], CAM_DATA_IN7_IDX);
    gpio_setup_in_s3(cfg->gpio_clk_in, CAM_PCLK_IDX);

    route_cam_control_idle();

    s_s3.dma_ready = (s_s3.dma_chan != nullptr);
}

static void start_dma_capture_s3_state(logic_analyzer_state_t *state, size_t capture_bytes)
{
    if (!state)
        return;

    s_s3.active_state = state;
    s_s3.eof_count = 0;
    s_s3.last_eof_desc = 0;
    s_s3.last_first_word = 0;
    state->dma_done = false;
    state->dma_desc_cur = 0;
    state->dma_received_count = 0;
    state->dma_filtered_count = 0;
    state->dma_desc_triggered = -1;

    for (size_t i = 0; i < state->dma_desc_count; ++i)
        memset(state->dma_buf[i], 0, state->dma_desc[i].size);

    if (!s_s3.dma_ready || !s_s3.dma_chan || !s_s3.dma_desc)
    {
        if (!s_warned_not_supported)
        {
            ESP_LOGW(S3_HAL_TAG, "ESP32-S3 GDMA backend is not ready; returning zeroed samples");
            s_warned_not_supported = true;
        }
        state->dma_done = true;
        return;
    }

    if (capture_bytes == 0)
        capture_bytes = state->dma_buf_width;

    cam_reset_and_fifo();
    LCD_CAM.lc_dma_int_clr.val = 0xFFFFFFFF;
    LCD_CAM.cam_ctrl.cam_update = 1;
    LCD_CAM.cam_ctrl.cam_vs_eof_en = 0;
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = capture_bytes - 1;

    configure_dma_descriptors(state);

    if (gdma_reset(s_s3.dma_chan) != ESP_OK || gdma_start(s_s3.dma_chan, reinterpret_cast<intptr_t>(s_s3.dma_desc)) != ESP_OK)
    {
        ESP_LOGE(S3_HAL_TAG, "Failed to start GDMA RX capture on ESP32-S3");
        state->dma_done = true;
        return;
    }

    route_cam_control_idle();
    cam_start_streaming();
    route_cam_control_active();
}

static void start_dma_capture_s3(void *ctx)
{
    logic_analyzer_state_t *state = get_logic_state(ctx);
    if (!state)
        return;
    start_dma_capture_s3_state(state, get_capture_byte_count(ctx));
}

bool init_esp32s3(const Config &cfg)
{
    s_cfg = cfg;
    s_cfg_valid = true;
    return true;
}

void start_esp32s3() {}

void stop_esp32s3()
{
    cam_stop_streaming();
    route_cam_control_idle();
    if (s_s3.dma_chan)
        gdma_stop(s_s3.dma_chan);
}

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
