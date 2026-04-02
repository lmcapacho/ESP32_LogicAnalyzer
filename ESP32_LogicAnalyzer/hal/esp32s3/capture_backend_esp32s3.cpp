#include "capture_backend_esp32s3.h"

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "../../LogicAnalyzer.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "hal/dma_types.h"
#include "soc/gdma_periph.h"
#include "soc/gdma_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/lcd_cam_struct.h"
#include "soc/periph_defs.h"
#if __has_include("esp_rom_gpio.h")
#include "esp_rom_gpio.h"
#elif __has_include("esp32s3/rom/gpio.h")
#include "esp32s3/rom/gpio.h"
#endif

namespace capture_backend_esp32s3 {

namespace {
constexpr const char *S3_HAL_TAG = "capture_backend_s3";
constexpr size_t kS3DmaChunkBytes = 4000;
constexpr int kConstOneInput = 0x38;
constexpr int kConstZeroInput = 0x3C;

typedef struct dma_descriptor_align8_s dmadesc_t;
struct dma_descriptor_align8_s {
    struct {
        uint32_t size : 12;
        uint32_t length : 12;
        uint32_t reversed24_27 : 4;
        uint32_t err_eof : 1;
        uint32_t reserved29 : 1;
        uint32_t suc_eof : 1;
        uint32_t owner : 1;
    } dw0;
    void *buffer;
    dmadesc_t *next;
    uint32_t free;
};
static_assert(sizeof(dmadesc_t) == 16, "ESP32-S3 DMA descriptor must be 16 bytes");

i2s_dma_hal::Config s_cfg = {};
bool s_cfg_valid = false;
bool s_warned_not_supported = false;

struct S3BackendState {
    dmadesc_t *dma_desc = nullptr;
    uint8_t **frame_buf = nullptr;
    size_t *frame_size = nullptr;
    size_t dma_desc_count = 0;
    size_t raw_byte_size = 0;
    int dma_num = -1;
    bool dma_ready = false;
    volatile bool capture_done = false;
    volatile size_t completed_desc_count = 0;
    volatile uint32_t eof_count = 0;
    volatile uintptr_t last_eof_desc = 0;
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

inline void route_cam_control_idle()
{
    esp_rom_gpio_connect_in_signal(kConstZeroInput, CAM_V_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(kConstOneInput, CAM_H_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(kConstOneInput, CAM_H_ENABLE_IDX, false);
}

inline void route_cam_control_start()
{
    esp_rom_gpio_connect_in_signal(kConstOneInput, CAM_V_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(kConstOneInput, CAM_H_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(kConstOneInput, CAM_H_ENABLE_IDX, false);
}

size_t calc_dma_desc_count(size_t raw_byte_size)
{
    return (raw_byte_size / kS3DmaChunkBytes) + 1;
}

size_t calc_dma_chunk_size(size_t raw_byte_size, size_t index, size_t desc_count)
{
    const size_t full_desc = raw_byte_size / kS3DmaChunkBytes;
    const size_t last_size = raw_byte_size % kS3DmaChunkBytes;
    if (index < full_desc)
        return kS3DmaChunkBytes;
    if (index + 1 == desc_count)
        return last_size;
    return 0;
}

void free_frame_storage()
{
    if (s_s3.frame_buf) {
        for (size_t i = 0; i < s_s3.dma_desc_count; ++i)
            heap_caps_free(s_s3.frame_buf[i]);
        heap_caps_free(s_s3.frame_buf);
        s_s3.frame_buf = nullptr;
    }
    if (s_s3.frame_size) {
        heap_caps_free(s_s3.frame_size);
        s_s3.frame_size = nullptr;
    }
    if (s_s3.dma_desc) {
        heap_caps_free(s_s3.dma_desc);
        s_s3.dma_desc = nullptr;
    }
    s_s3.dma_desc_count = 0;
    s_s3.raw_byte_size = 0;
}

esp_err_t ensure_frame_storage(size_t raw_byte_size)
{
    if (raw_byte_size == 0)
        return ESP_ERR_INVALID_ARG;

    const size_t desc_count = calc_dma_desc_count(raw_byte_size);
    if (s_s3.dma_desc && s_s3.frame_buf && s_s3.frame_size &&
        s_s3.dma_desc_count == desc_count && s_s3.raw_byte_size == raw_byte_size)
        return ESP_OK;

    free_frame_storage();

    s_s3.dma_desc_count = desc_count;
    s_s3.raw_byte_size = raw_byte_size;
    s_s3.frame_buf = static_cast<uint8_t **>(heap_caps_calloc(desc_count, sizeof(uint8_t *), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    s_s3.frame_size = static_cast<size_t *>(heap_caps_calloc(desc_count, sizeof(size_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    s_s3.dma_desc = static_cast<dmadesc_t *>(heap_caps_calloc(desc_count, sizeof(dmadesc_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (!s_s3.frame_buf || !s_s3.frame_size || !s_s3.dma_desc)
        return ESP_ERR_NO_MEM;

    for (size_t i = 0; i < desc_count; ++i) {
        s_s3.frame_size[i] = calc_dma_chunk_size(raw_byte_size, i, desc_count);
        if (s_s3.frame_size[i] == 0)
            continue;
        s_s3.frame_buf[i] = static_cast<uint8_t *>(heap_caps_malloc(s_s3.frame_size[i], MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        if (!s_s3.frame_buf[i])
            return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void configure_dma_descriptors()
{
    if (!s_s3.dma_desc || !s_s3.frame_buf || !s_s3.frame_size)
        return;

    for (size_t i = 0; i < s_s3.dma_desc_count; ++i) {
        dmadesc_t &desc = s_s3.dma_desc[i];
        const size_t chunk = s_s3.frame_size[i];
        desc.dw0.size = chunk;
        desc.dw0.length = chunk;
        desc.dw0.err_eof = 0;
        desc.dw0.suc_eof = 0;
        desc.dw0.owner = 1;
        desc.buffer = chunk ? s_s3.frame_buf[i] : nullptr;
        desc.next = (i + 1 < s_s3.dma_desc_count) ? &s_s3.dma_desc[i + 1] : nullptr;
        desc.free = 0;
    }
}

void clear_frame_buffers()
{
    if (!s_s3.frame_buf || !s_s3.frame_size)
        return;
    for (size_t i = 0; i < s_s3.dma_desc_count; ++i) {
        if (s_s3.frame_buf[i] && s_s3.frame_size[i])
            memset(s_s3.frame_buf[i], 0, s_s3.frame_size[i]);
    }
}

void copy_frame_to_logic_state_impl(logic_analyzer_state_t *state)
{
    if (!s_s3.frame_buf || !s_s3.frame_size || !state || !state->dma_buf)
        return;
    const size_t copy_desc = (state->dma_desc_count < s_s3.dma_desc_count) ? state->dma_desc_count : s_s3.dma_desc_count;
    for (size_t i = 0; i < copy_desc; ++i) {
        if (!s_s3.frame_buf[i] || !s_s3.frame_size[i])
            continue;
        uint8_t *dst = reinterpret_cast<uint8_t *>(state->dma_buf[i]);
        uint8_t *src = s_s3.frame_buf[i];
        for (size_t j = 0; j < s_s3.frame_size[i]; ++j)
            dst[j] = src[j] & 0x01;
    }
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

bool dma_init_channel()
{
    periph_module_enable(PERIPH_GDMA_MODULE);

    if (s_s3.dma_num >= 0)
        return true;

    for (int x = SOC_GDMA_PAIRS_PER_GROUP - 1; x >= 0; --x) {
        if (GDMA.channel[x].in.link.addr == 0) {
            s_s3.dma_num = x;
            break;
        }
    }
    if (s_s3.dma_num < 0)
        return false;

    GDMA.channel[s_s3.dma_num].in.int_clr.val = ~0U;
    GDMA.channel[s_s3.dma_num].in.int_ena.val = 0;
    GDMA.channel[s_s3.dma_num].in.conf0.val = 0;
    GDMA.channel[s_s3.dma_num].in.conf1.val = 0;
    GDMA.channel[s_s3.dma_num].in.conf0.in_rst = 1;
    GDMA.channel[s_s3.dma_num].in.conf0.in_rst = 0;
    GDMA.channel[s_s3.dma_num].in.conf0.indscr_burst_en = 1;
    GDMA.channel[s_s3.dma_num].in.conf0.in_data_burst_en = 1;
    GDMA.channel[s_s3.dma_num].in.conf1.in_check_owner = 0;
    GDMA.channel[s_s3.dma_num].in.peri_sel.sel = 5;
    return true;
}

void update_capture_state_from_regs()
{
    if (s_s3.dma_num < 0)
        return;

    auto &in = GDMA.channel[s_s3.dma_num].in;
    if (in.int_raw.in_suc_eof || in.int_st.in_suc_eof) {
        s_s3.eof_count++;
        s_s3.last_eof_desc = in.suc_eof_des_addr;
        s_s3.completed_desc_count = s_s3.dma_desc_count;
        if (s_s3.frame_buf && s_s3.frame_buf[0])
            s_s3.last_first_word = reinterpret_cast<uint32_t *>(s_s3.frame_buf[0])[0];
        s_s3.capture_done = true;
        in.int_clr.in_suc_eof = 1;
    } else if (in.int_raw.in_dscr_empty || in.int_st.in_dscr_empty) {
        s_s3.eof_count++;
        s_s3.last_eof_desc = in.suc_eof_des_addr;
        s_s3.completed_desc_count = s_s3.dma_desc_count;
        if (s_s3.frame_buf && s_s3.frame_buf[0])
            s_s3.last_first_word = reinterpret_cast<uint32_t *>(s_s3.frame_buf[0])[0];
        s_s3.capture_done = true;
        in.int_clr.in_dscr_empty = 1;
    }
}
} // namespace

static void i2s_parallel_setup_s3(const i2s_parallel_config_t *cfg)
{
    if (!cfg)
        return;

    periph_module_enable(PERIPH_LCD_CAM_MODULE);
    LCD_CAM.cam_ctrl.val = 0;
    LCD_CAM.cam_ctrl1.val = 0;
    LCD_CAM.cam_rgb_yuv.val = 0;
    LCD_CAM.lc_dma_int_ena.val = 0;
    LCD_CAM.lc_dma_int_clr.val = 0xFFFFFFFF;

    cam_reset_and_fifo();

    LCD_CAM.cam_ctrl.cam_clk_sel = 3;
    LCD_CAM.cam_ctrl.cam_clkm_div_num = 4;
    LCD_CAM.cam_ctrl.cam_clkm_div_a = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_b = 0;
    LCD_CAM.cam_ctrl.cam_stop_en = 0;
    LCD_CAM.cam_ctrl.cam_vs_eof_en = 1;
    LCD_CAM.cam_ctrl.cam_update = 1;

    LCD_CAM.cam_ctrl1.cam_clk_inv = 0;
    LCD_CAM.cam_ctrl1.cam_2byte_en = 0;
    LCD_CAM.cam_ctrl1.cam_de_inv = 0;
    LCD_CAM.cam_ctrl1.cam_hsync_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vsync_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vh_de_mode_en = 0;
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
    esp_rom_gpio_pad_select_gpio(cfg->gpio_clk_in);
    gpio_set_direction((gpio_num_t)cfg->gpio_clk_in, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode((gpio_num_t)cfg->gpio_clk_in, GPIO_FLOATING);
    esp_rom_gpio_connect_out_signal(cfg->gpio_clk_in, CAM_CLK_IDX, false, false);
    esp_rom_gpio_connect_in_signal(cfg->gpio_clk_in, CAM_PCLK_IDX, false);

    route_cam_control_idle();
    s_s3.dma_ready = dma_init_channel();
}

static void start_dma_capture_s3(size_t capture_bytes)
{
    s_s3.eof_count = 0;
    s_s3.last_eof_desc = 0;
    s_s3.last_first_word = 0;
    s_s3.capture_done = false;
    s_s3.completed_desc_count = 0;

    clear_frame_buffers();

    if (!s_s3.dma_ready || s_s3.dma_num < 0 || !s_s3.dma_desc) {
        if (!s_warned_not_supported) {
            ESP_LOGW(S3_HAL_TAG, "ESP32-S3 GDMA backend is not ready; returning zeroed samples");
            s_warned_not_supported = true;
        }
        s_s3.capture_done = true;
        return;
    }

    if (capture_bytes == 0)
        capture_bytes = s_s3.raw_byte_size;
    (void)capture_bytes;

    cam_reset_and_fifo();
    configure_dma_descriptors();

    auto &in = GDMA.channel[s_s3.dma_num].in;
    in.int_clr.val = ~0U;
    in.int_ena.val = 0;
    in.conf0.in_rst = 1;
    in.conf0.in_rst = 0;
    in.conf1.in_check_owner = 0;
    in.peri_sel.sel = 5;

    LCD_CAM.cam_ctrl.cam_vs_eof_en = 1;
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = 64;
    LCD_CAM.cam_ctrl.cam_update = 1;

    in.link.addr = (reinterpret_cast<uint32_t>(&s_s3.dma_desc[0])) & 0xFFFFF;
    in.int_ena.in_suc_eof = 1;
    in.int_clr.in_suc_eof = 1;
    in.int_ena.in_dscr_empty = 1;
    in.int_clr.in_dscr_empty = 1;
    in.link.stop = 0;
    in.link.start = 1;

    LCD_CAM.cam_ctrl1.cam_start = 1;
    route_cam_control_start();
}

bool init(const i2s_dma_hal::Config &cfg)
{
    s_cfg = cfg;
    s_cfg_valid = true;
    return true;
}

void start() {}

void stop()
{
    LCD_CAM.cam_ctrl1.cam_start = 0;
    route_cam_control_idle();
    if (s_s3.dma_num >= 0) {
        auto &in = GDMA.channel[s_s3.dma_num].in;
        in.link.stop = 1;
        in.link.start = 0;
        in.link.addr = 0;
        in.int_ena.in_suc_eof = 0;
        in.int_clr.in_suc_eof = 1;
        in.int_ena.in_dscr_empty = 0;
        in.int_clr.in_dscr_empty = 1;
    }
}

bool configure(const i2s_parallel_config_t *cfg, int raw_byte_size)
{
    if (!cfg || !s_cfg_valid)
        return false;
    if (ensure_frame_storage(raw_byte_size) != ESP_OK)
        return false;
    configure_dma_descriptors();
    i2s_parallel_setup_s3(cfg);
    return s_s3.dma_ready;
}

Result capture(size_t capture_bytes, uint32_t timeout_ms)
{
    start_dma_capture_s3(capture_bytes);

    const unsigned long start_ms = millis();
    while (!s_s3.capture_done) {
        update_capture_state_from_regs();
        if (millis() - start_ms > timeout_ms) {
            ESP_LOGW(S3_HAL_TAG, "ESP32-S3 capture timeout waiting for DMA EOF");
            s_s3.capture_done = true;
            break;
        }
        delay(1);
    }

    stop();
    return Result{ s_s3.capture_done, s_s3.completed_desc_count };
}

void copy_to_logic_state(logic_analyzer_state_t *state)
{
    copy_frame_to_logic_state_impl(state);
}

DebugSnapshot debug_snapshot()
{
    return DebugSnapshot{
        s_s3.dma_ready,
        s_s3.eof_count,
        s_s3.last_eof_desc,
        s_s3.last_first_word,
    };
}

} // namespace capture_backend_esp32s3

#endif
