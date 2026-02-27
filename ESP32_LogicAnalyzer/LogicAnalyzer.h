#include <Arduino.h>
#include "esp_err.h"
#include "esp_intr_alloc.h"
#if __has_include("soc/lldesc.h")
#include "soc/lldesc.h"
#elif __has_include("esp_private/dma_types.h")
#include "esp_private/dma_types.h"
#elif __has_include("hal/dma_types.h")
#include "hal/dma_types.h"
#else
#include "rom/lldesc.h"
#endif

#include "LogicAnalyzerConfig.h"

#define I2S_PARALLEL_BITS I2S_PARALLEL_BITS_16
#define DMA_MAX (4096 - 4)

/* XON/XOFF are not supported. */
#define SUMP_RESET 0x00
#define SUMP_ARM 0x01
#define SUMP_QUERY 0x02
#define SUMP_XON 0x11
#define SUMP_XOFF 0x13

/* mask & values used, config ignored. only stage0 supported */
#define SUMP_TRIGGER_MASK_CH_A 0xC0
#define SUMP_TRIGGER_MASK_CH_B 0xC4
#define SUMP_TRIGGER_MASK_CH_C 0xC8
#define SUMP_TRIGGER_MASK_CH_D 0xCC

#define SUMP_TRIGGER_VALUES_CH_A 0xC1
#define SUMP_TRIGGER_VALUES_CH_B 0xC5
#define SUMP_TRIGGER_VALUES_CH_C 0xC9
#define SUMP_TRIGGER_VALUES_CH_D 0xCD

#define SUMP_TRIGGER_CONFIG_CH_A 0xC2
#define SUMP_TRIGGER_CONFIG_CH_B 0xC6
#define SUMP_TRIGGER_CONFIG_CH_C 0xCA
#define SUMP_TRIGGER_CONFIG_CH_D 0xCE

/* Most flags (except RLE) are ignored. */
#define SUMP_SET_DIVIDER 0x80
#define SUMP_SET_READ_DELAY_COUNT 0x81
#define SUMP_SET_FLAGS 0x82
#define SUMP_SET_RLE 0x0100

/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST 0x03
#define SUMP_GET_METADATA 0x04

#define MAX_CAPTURE_SIZE CAPTURE_SIZE

typedef enum
{
    I2S_PARALLEL_BITS_8 = 8,
    I2S_PARALLEL_BITS_16 = 16,
    I2S_PARALLEL_BITS_32 = 32,
} i2s_parallel_cfg_bits_t;

typedef enum
{
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */

    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, .
     *
     * but appears as 00 s2 00 s1, 00 s4 00 s3 at DMA buffer somehow...
     *
     */

    SM_0A00_0B00 = 3,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
} i2s_sampling_mode_t;

typedef struct
{
    void *memory;
    size_t size;
} i2s_parallel_buffer_desc_t;

typedef struct i2s_parallel_config_t
{
    int8_t gpio_bus[16];
    int8_t gpio_clk_in;
    int8_t gpio_clk_out;

    int clkspeed_hz;
    i2s_parallel_cfg_bits_t bits;
    i2s_parallel_buffer_desc_t *buf;
} i2s_parallel_config_t;

typedef struct
{
    volatile lldesc_t *dmadesc;
    int desccount;
} i2s_parallel_state_t;

typedef union
{
    struct
    {
        uint8_t sample2;
        uint8_t unused2;
        uint8_t sample1;
        uint8_t unused1;
    };
    struct
    {
        uint16_t val2;
        uint16_t val1;
    };
    uint32_t val;
} dma_elem_t;

typedef struct
{
    lldesc_t *dma_desc;
    dma_elem_t **dma_buf;
    bool dma_done;
    size_t dma_desc_count;
    size_t dma_desc_cur;
    int dma_desc_triggered;
    size_t dma_received_count;
    size_t dma_filtered_count;
    size_t dma_buf_width;
    size_t dma_sample_count;
    size_t dma_val_per_desc;
    size_t dma_sample_per_desc;
    i2s_sampling_mode_t sampling_mode;
    //    dma_filter_t dma_filter;
    intr_handle_t i2s_intr_handle;
    //    QueueHandle_t data_ready;
    //    SemaphoreHandle_t frame_ready;
    //    TaskHandle_t dma_filter_task;
} logic_analyzer_state_t;

void IRAM_ATTR i2s_wrapper(void *arg);

class LogicAnalyzer
{
public:
    void begin(void);
    void handleCommand(int cmd);
    void i2s_isr(void *arg);

private:
    uint8_t channels_to_read = 3;

    i2s_parallel_buffer_desc_t bufdesc;

    int8_t rle_process = -1;
    uint8_t rle_buff[RLE_BUFFER_SIZE];
    uint8_t *rle_buff_p;
    uint8_t *rle_buff_end;
    uint8_t rle_sample_counter;
    uint32_t rle_total_sample_counter;
    uint8_t rle_value_holder;
    i2s_parallel_state_t *i2s_state[2] = {NULL, NULL};
    logic_analyzer_state_t *s_state;
    bool capture_backend_ready = true;

    uint32_t time_debug_indice_dma[10];
    uint16_t time_debug_indice_dma_p = 0;

    uint32_t time_debug_indice_rle[10];
    uint16_t time_debug_indice_rle_p = 0;

    uint16_t time_debug_indice_lenght = 10;

    int stop_at_desc = -1;
    unsigned int logicIndex = 0;
    unsigned int triggerIndex = 0;
    uint32_t readCount = CAPTURE_SIZE;
    unsigned int delayCount = 0;
    uint16_t trigger = 0;
    uint16_t trigger_values = 0;
    unsigned int useMicro = 0;
    unsigned int delayTime = 0;
    unsigned long divider = 0;
    boolean rleEnabled = 0;
    uint32_t clock_per_read = 0;

    void captureMilli(void);
    void getCmd(byte *cmdBytes);
    void get_metadata(void);
    void setupDelay();

    void gpio_setup_in(int gpio, int sig, int inv);
    void enable_out_clock(uint32_t freq_in_hz);
    
    void dma_desc_deinit();
    esp_err_t dma_desc_init(int raw_byte_size);
    void start_dma_capture(void);
    void dma_serializer(dma_elem_t *dma_buffer);
    void dmabuff_compresser_ch1(uint8_t *dma_buffer);
    void dmabuff_compresser_ch2(uint8_t *dma_buffer);

    void i2s_conf_reset();
    void i2s_parallel_setup(const i2s_parallel_config_t *cfg);

    static esp_err_t hal_dma_desc_init_bridge(void *ctx, int raw_byte_size);
    static void hal_i2s_parallel_setup_bridge(void *ctx, const i2s_parallel_config_t *cfg);
    static void hal_start_dma_capture_bridge(void *ctx);

    bool rle_init(void);
    void fast_rle_block_encode_asm_8bit_ch1(uint8_t *dma_buffer, int sample_size);
    void fast_rle_block_encode_asm_8bit_ch2(uint8_t *dma_buffer, int sample_size);
    void fast_rle_block_encode_asm_16bit(uint8_t *dma_buffer, int sample_size);
};

uint16_t buff_process_trigger_1(uint16_t *buff, int size, bool printit = true);
uint16_t buff_process_trigger_0(uint16_t *buff, int size, bool printit = true);
int calc_needed_dma_descs_for(i2s_parallel_buffer_desc_t *desc);
