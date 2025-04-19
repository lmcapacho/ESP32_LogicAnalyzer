#include "LogicAnalyzer.h"

bool LogicAnalyzer::rle_init(void)
{
    rle_buff_p = 0;
    rle_sample_counter = 0;
    rle_total_sample_counter = 0;
    rle_value_holder = 0;
    rle_process = -1;

    rle_buff_p = rle_buff;
    rle_buff_end = rle_buff + RLE_BUFFER_SIZE - 4;

    memset(rle_buff, 0x00, RLE_BUFFER_SIZE);
    return true;
}

void LogicAnalyzer::fast_rle_block_encode_asm_8bit_ch1(uint8_t *dma_buffer, int sample_size)
{ // size, not count
    uint8_t *desc_buff_end = dma_buffer;
    unsigned clocka = 0, clockb = 0;

    /* We have to encode RLE samples quick.
     * Each sample need to be encoded under 12 clocks @240Mhz CPU
     * for capture 20Mhz sampling speed.
     */

    /* expected structure of DMA memory    : 00s1,00s2,00s3,00s4
     * actual data structure of DMA memory : 00s2,00s1,00s4,00s3
     */

    int dword_count = (sample_size / 4) - 1;

    clocka = xthal_get_ccount();

    /* No, Assembly is not that hard. You are just too lazzy. */

    // "a4"=&dma_buffer, "a5"=dword_count, "a6"=&rle_buff_p:

    __asm__ __volatile__(
        "memw \n"
        "l32i a4, %0, 0        \n"       // Load store dma_buffer address
        "l32i a6, %2, 0        \n"       // Load store rle_buffer address
        "l8ui a8, a4, 2        \n"       // a8 as rle_val (#2 is first)
        "l8ui a9, a4, 0        \n"       // a9 as new_val
        "movi a7, 0xFF         \n"       // store max RLE sample
        "movi a5, 0            \n"       // init rle_counter
        "movi a10, 0x80        \n"       // init rle_masks
        "movi a11, 0x7F        \n"       // init rle_masks
        "beq  a9, a8, rle_0          \n" // rle_val == new_val skip

#if not ALLOW_ZERO_RLE
        "and  a8, a8, a11      \n" // a11=0x7F
#endif
        "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = rle_counter //not needed
        "addi a6, a6, 1        \n" // rle_buff_p ++              //not needed
#endif
        "movi a5, -1           \n" // rle_counter=-1
        "addi a6, a6, 1        \n" // rle_buff_p ++
        "mov  a8, a9           \n" // rle_val = new_val

        "rle_0:                \n"
        "addi a5, a5, 1        \n"       // rle_counter++
        "loopnez %1, rle_loop_end    \n" // Prepare zero-overhead loop
        "loopStart:            \n"
        "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

        "rle_1_start:          \n"
        "l8ui a9, a4, 2        \n"       // a8 as rle_val (#2 is first)
        "beq  a9, a8, rle_1_end      \n" // rle_val == new_val skip

        "bltui a5, 128, rle_1_add    \n" // if count >= 128 branch
        "rle_1_127:            \n"
#if not ALLOW_ZERO_RLE
        "and  a8, a8, a11      \n" // a11=0x7F
#endif
        "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
        "movi a7, 0xFF         \n"
        "s8i  a7, a6, 1        \n"       // rle_buff_p+1 = 127 as count
        "addi a6, a6, 2        \n"       // rle_buff_p ++2
        "addi a5, a5, -128     \n"       // count=-127
        "bgeui a5, 128, rle_1_127    \n" // if count >= 128 branch

        "rle_1_add:            \n"
#if ALLOW_ZERO_RLE                 // 4000 to 3140 bytes & 27219 clocks
        "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
        "addi a6, a6, 2        \n" // rle_buff_p ++2
#else                              // 4000 to 1988 bytes & 34348 clocks
        "and  a8, a8, a11      \n"       // a11=0x7F
        "s8i  a8, a6, 0        \n"       // rle_buff_p = rle_val & 0x7F;
        "beqz a5, rle_1_skip         \n" // if count == 0 , skip
        "or   a5, a5, a10      \n"       // a10=0x80
        "s8i  a5, a6, 1        \n"       // rle_buff_p+1 = count | 0x80;
        "addi a6, a6, 1        \n"       // rle_buff_p ++
        "rle_1_skip:           \n"
        "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
        "mov  a8, a9           \n" // rle_val = new_val
        "movi a5, -1           \n" // rle_counter=-1  , will be 0 at next instruction.

        "rle_1_end:            \n"
        "addi a5, a5, 1        \n" // rle_counter++

        "rle_2_start:          \n"
        "l8ui a9, a4, 0        \n"       // a9 as rle_val (#0 is second)
        "beq  a9, a8, rle_2_end      \n" // rle_val == new_val continue

        "bltui a5, 128, rle_2_add    \n" // if count >= 128 branch
        "rle_2_127:            \n"
#if not ALLOW_ZERO_RLE
        "and  a8, a8, a11      \n" // a11=0x7F
#endif
        "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
        "movi a7, 0xFF         \n"
        "s8i  a7, a6, 1        \n"       // rle_buff_p+1 = 127 as count
        "addi a6, a6, 2        \n"       // rle_buff_p ++2
        "addi a5, a5, -128     \n"       // count=-127
        "bgeui a5, 128, rle_2_127    \n" // if count >= 128 branch

        "rle_2_add:            \n"
#if ALLOW_ZERO_RLE
        "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
        "addi a6, a6, 2        \n" // rle_buff_p ++2
#else
        "and  a8, a8, a11      \n" // a11=0x7F
        "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
        "beqz a5, rle_2_skip         \n"
        "or   a5, a5, a10      \n" // a10=0x80
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
        "addi a6, a6, 1        \n" // rle_buff_p ++
        "rle_2_skip:           \n"
        "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
        "mov  a8, a9           \n" // rle_val = new_val
        "movi a5, -1           \n" // rle_counter=-1
        "rle_2_end:            \n"
        "addi a5, a5, 1        \n" // rle_counter++

        "rle_loop_end:              \n"

        "bltui a5, 128, rle_end_add  \n" // if count >= 128 branch
        "rle_end_127:          \n"
#if not ALLOW_ZERO_RLE
        "and  a8, a8, a11      \n" // a11=0x7F
#endif
        "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
        "movi a7, 0xFF         \n"
        "s8i  a7, a6, 1        \n"       // rle_buff_p+1 = 127 as count
        "addi a6, a6, 2        \n"       // rle_buff_p ++2
        "addi a5, a5, -128     \n"       // count=-127
        "bgeui a5, 128, rle_end_127  \n" // if count >= 128 branch

        "rle_end_add:          \n"
#if ALLOW_ZERO_RLE
        "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
        "addi a6, a6, 2        \n" // rle_buff_p ++2
#else
        "and  a8, a8, a11      \n" //
        "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
        "beqz a5, rle_end_skip       \n"
        "or   a5, a5, a10      \n" //
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
        "addi a6, a6, 1        \n" // rle_buff_p ++
        "rle_end_skip:         \n"
        "addi a6, a6, 1        \n" // rle_buff_p ++
#endif

        "exit:                 \n"
        "s32i a6, %2, 0        \n" // update rle_buff_p value

        :
        : "a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p) : "a4", "a5", "a6", "a7", "a8", "a9", "a10", "a11", "memory");

    clockb = xthal_get_ccount();
    // if(time_debug_indice_rle_p < time_debug_indice_lenght)
    //   time_debug_indice_rle[time_debug_indice_rle_p++]=clockb;

    //   Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
    //   Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
    //   Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
    ESP_LOGD(TAG, "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff));
}

void LogicAnalyzer::fast_rle_block_encode_asm_8bit_ch2(uint8_t *dma_buffer, int sample_size)
{ // size, not count
    uint8_t *desc_buff_end = dma_buffer;
    unsigned clocka = 0, clockb = 0;

    /* We have to encode RLE samples quick.
     * Each sample need to be encoded under 12 clocks @240Mhz CPU
     * for capture 20Mhz sampling speed.
     */

    /* expected structure of DMA memory    : 00s1,00s2,00s3,00s4
     * actual data structure of DMA memory : 00s2,00s1,00s4,00s3
     */

    int dword_count = (sample_size / 4) - 1;

    clocka = xthal_get_ccount();

    /* No, Assembly is not that hard. You are just too lazzy. */

    __asm__ __volatile__(
        "memw \n"
        "l32i a4, %0, 0        \n"       // Load store dma_buffer address
        "l32i a6, %2, 0        \n"       // Load store rle_buffer address
        "l8ui a8, a4, 3        \n"       // a8 as rle_val (#2 is first)
        "l8ui a9, a4, 1        \n"       // a9 as new_val
        "movi a7, 0xFF         \n"       // store max RLE sample
        "movi a5, 0            \n"       // init rle_counter
        "movi a10, 0x80        \n"       // init rle_masks
        "movi a11, 0x7F        \n"       // init rle_masks
        "beq  a9, a8, rle_0_ch2      \n" // rle_val == new_val skip

#if not ALLOW_ZERO_RLE
        "and  a8, a8, a11      \n" // a11=0x7F
#endif
        "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = rle_counter //not needed
        "addi a6, a6, 1        \n" // rle_buff_p ++              //not needed
#endif
        "movi a5, -1           \n" // rle_counter=-1
        "addi a6, a6, 1        \n" // rle_buff_p ++
        "mov  a8, a9           \n" // rle_val = new_val

        "rle_0_ch2:            \n"
        "addi a5, a5, 1        \n"           // rle_counter++
        "loopnez %1, rle_loop_end_ch2    \n" // Prepare zero-overhead loop
        "loopStart_ch2:            \n"
        "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

        "rle_1_start_ch2:          \n"
        "l8ui a9, a4, 3        \n"           // a8 as rle_val (#2 is first)
        "beq  a9, a8, rle_1_end_ch2      \n" // rle_val == new_val skip

        "bltui a5, 128, rle_1_add_ch2    \n" // if count >= 128 branch
        "rle_1_127_ch2:            \n"
#if not ALLOW_ZERO_RLE
        "and  a8, a8, a11      \n" // a11=0x7F
#endif
        "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
        "movi a7, 0xFF         \n"
        "s8i  a7, a6, 1        \n"           // rle_buff_p+1 = 127 as count
        "addi a6, a6, 2        \n"           // rle_buff_p ++2
        "addi a5, a5, -128     \n"           // count=-127
        "bgeui a5, 128, rle_1_127_ch2    \n" // if count >= 128 branch

        "rle_1_add_ch2:            \n"
#if ALLOW_ZERO_RLE                 // 4000 to 3140 bytes & 27219 clocks
        "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
        "addi a6, a6, 2        \n" // rle_buff_p ++2
#else                              // 4000 to 1988 bytes & 34348 clocks
        "and  a8, a8, a11      \n"           // a11=0x7F
        "s8i  a8, a6, 0        \n"           // rle_buff_p = rle_val & 0x7F;
        "beqz a5, rle_1_skip_ch2         \n" // if count == 0 , skip
        "or   a5, a5, a10      \n"           // a10=0x80
        "s8i  a5, a6, 1        \n"           // rle_buff_p+1 = count | 0x80;
        "addi a6, a6, 1        \n"           // rle_buff_p ++
        "rle_1_skip_ch2:           \n"
        "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
        "mov  a8, a9           \n" // rle_val = new_val
        "movi a5, -1           \n" // rle_counter=-1  , will be 0 at next instruction.

        "rle_1_end_ch2:            \n"
        "addi a5, a5, 1        \n" // rle_counter++

        "rle_2_start_ch2:          \n"
        "l8ui a9, a4, 1        \n"           // a9 as rle_val (#0 is second)
        "beq  a9, a8, rle_2_end_ch2      \n" // rle_val == new_val continue

        "bltui a5, 128, rle_2_add_ch2   \n" // if count >= 128 branch
        "rle_2_127_ch2:            \n"
#if not ALLOW_ZERO_RLE
        "and  a8, a8, a11      \n" // a11=0x7F
#endif
        "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
        "movi a7, 0xFF         \n"
        "s8i  a7, a6, 1        \n"           // rle_buff_p+1 = 127 as count
        "addi a6, a6, 2        \n"           // rle_buff_p ++2
        "addi a5, a5, -128     \n"           // count=-127
        "bgeui a5, 128, rle_2_127_ch2    \n" // if count >= 128 branch

        "rle_2_add_ch2:            \n"
#if ALLOW_ZERO_RLE
        "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
        "addi a6, a6, 2        \n" // rle_buff_p ++2
#else
        "and  a8, a8, a11      \n" // a11=0x7F
        "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
        "beqz a5, rle_2_skip_ch2         \n"
        "or   a5, a5, a10      \n" // a10=0x80
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
        "addi a6, a6, 1        \n" // rle_buff_p ++
        "rle_2_skip_ch2:           \n"
        "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
        "mov  a8, a9           \n" // rle_val = new_val
        "movi a5, -1           \n" // rle_counter=-1
        "rle_2_end_ch2:            \n"
        "addi a5, a5, 1        \n" // rle_counter++

        "rle_loop_end_ch2:              \n"

        "bltui a5, 128, rle_end_add_ch2  \n" // if count >= 128 branch
        "rle_end_127_ch2:          \n"
#if not ALLOW_ZERO_RLE
        "and  a8, a8, a11      \n" // a11=0x7F
#endif
        "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
        "movi a7, 0xFF         \n"
        "s8i  a7, a6, 1        \n"           // rle_buff_p+1 = 127 as count
        "addi a6, a6, 2        \n"           // rle_buff_p ++2
        "addi a5, a5, -128     \n"           // count=-127
        "bgeui a5, 128, rle_end_127_ch2  \n" // if count >= 128 branch

        "rle_end_add_ch2:          \n"
#if ALLOW_ZERO_RLE
        "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
        "addi a6, a6, 2        \n" // rle_buff_p ++2
#else
        "and  a8, a8, a11      \n" //
        "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
        "beqz a5, rle_end_skip_ch2       \n"
        "or   a5, a5, a10      \n" //
        "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
        "addi a6, a6, 1        \n" // rle_buff_p ++
        "rle_end_skip_ch2:         \n"
        "addi a6, a6, 1        \n" // rle_buff_p ++
#endif

        "exit_ch2:                 \n"
        "s32i a6, %2, 0        \n" // update rle_buff_p value

        :
        : "a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p) : "a4", "a5", "a6", "a7", "a8", "a9", "a10", "a11", "memory");

    clockb = xthal_get_ccount();

    //   Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
    //   Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
    //   Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
    ESP_LOGD(TAG, "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff));
}

void LogicAnalyzer::fast_rle_block_encode_asm_16bit(uint8_t *dma_buffer, int sample_size)
{ // size, not count
    // uint8_t *desc_buff_end=dma_buffer;
    // uint32_t clocka=0,clockb=0;

    /* We have to encode RLE samples quick.
     * Each sample need to be encoded under 12 clocks @240Mhz CPU
     * for capture 20Mhz sampling speed.
     */

    /* expected structure of DMA memory    : S1s1,S2s2,S3s3,S4s4
     * actual data structure of DMA memory : S2s2,S1s1,S4s4,S3s3
     */

    int dword_count = (sample_size / 4) - 1;

    // clocka = xthal_get_ccount();

    /* No, Assembly is not that hard. You are just too lazzy. */

    __asm__ __volatile__(
        "l32i a4, %0, 0        \n" // Load store dma_buffer address
        "l32i a6, %2, 0        \n" // Load store rle_buffer address
        "l16ui a8, a4, 2       \n" // a8 as rle_val #2 is first   a8=&(dma_buffer+2)
        "l16ui a9, a4, 0       \n" // a9 as new_val               a9=&dma_buffer
        "movi a7, 0xFF         \n" // store max RLE sample
        "movi a5, 0            \n" // init rle_counter
        "movi a10, 0x8000      \n" // init rle_masks for count
        "movi a11, 0x7FFF      \n" // init rle_masks for data

        "beq  a9, a8, rle_0_16       \n" // rle_val == new_val skip

#if not ALLOW_ZERO_RLE
        "and  a8, a8, a11      \n" // a11=0x7FFF
#endif
        "s16i  a8, a6, 0       \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
        "s16i  a5, a6, 2       \n" // rle_buff_p+2 = rle_counter //not needed
        "addi a6, a6, 2        \n" // rle_buff_p +=2             //not needed
#endif
        "movi a5, -1           \n" // rle_counter=-1
        "addi a6, a6, 2        \n" // rle_buff_p +=2
        "mov  a8, a9           \n" // rle_val = new_val

        "rle_0_16:             \n"
        "addi a5, a5, 1        \n"       // rle_counter++
        "loopnez %1, rle_loop_end_16 \n" // Prepare zero-overhead loop
        "loopStart_16:         \n"
        "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

        "rle_1_start_16:       \n"
        "l16ui a9, a4, 2       \n"       // a9 as rle_val #2 is first
        "beq  a9, a8, rle_1_end_16   \n" // rle_val == new_val skip

        "rle_1_127_16:         \n" // not needed

        "rle_1_add_16:         \n"
#if ALLOW_ZERO_RLE                 // 4000 to 3140 bytes & 22.2k clocks
        "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
        "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
        "addi a6, a6, 4        \n" // rle_buff_p ++4
#else                              // 4000 to 1988 bytes & 34348 clocks
        "and  a8, a8, a11      \n" // * 0x7F
        "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
        "beqz a5, rle_1_skip_16      \n"
        "or   a5, a5, a10      \n" // *
        "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
        "addi a6, a6, 2        \n" // rle_buff_p --
        "rle_1_skip_16:        \n"
        "addi a6, a6, 2        \n" // rle_buff_p --
#endif
        "mov  a8, a9           \n" // rle_val = new_val
        "movi a5, -1           \n" // rle_counter=-1

        "rle_1_end_16:         \n"
        "addi a5, a5, 1        \n" // rle_counter++

        "rle_2_start_16:       \n"
        "l16ui a9, a4, 0       \n"       // a9 as rle_val #0 is second
        "beq  a9, a8, rle_2_end_16   \n" // rle_val == new_val continue

        "rle_2_add_16:         \n"
#if ALLOW_ZERO_RLE
        "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
        "s16i  a5, a6, 2       \n" // *rle_buff_p+2 = count;
        "addi a6, a6, 4        \n" // rle_buff_p ++4
#else
        "and  a8, a8, a11      \n" // *
        "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
        "beqz a5, rle_2_skip_16      \n"
        "or   a5, a5, a10      \n" // *
        "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
        "addi a6, a6, 2        \n" // rle_buff_p --
        "rle_2_skip_16:        \n"
        "addi a6, a6, 2        \n" // rle_buff_p --
#endif
        "mov  a8, a9           \n" // rle_val = new_val
        "movi a5, -1           \n" // rle_counter=-1
        "rle_2_end_16:         \n"
        "addi a5, a5, 1        \n" // rle_counter++

        "rle_loop_end_16:      \n"

        "rle_end_add_16:       \n"
#if ALLOW_ZERO_RLE
        "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
        "s16i  a5, a6, 2       \n" // *rle_buff_p+2 = count;
        "addi a6, a6, 4        \n" // rle_buff_p ++4
#else
        "and  a8, a8, a11      \n" // *
        "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
        "beqz a5, rle_end_skip_16    \n"
        "or   a5, a5, a10      \n" // *
        "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
        "addi a6, a6, 2        \n" // rle_buff_p --
        "rle_end_skip_16:      \n"
        "addi a6, a6, 2        \n" // rle_buff_p --
#endif

        "exit_16:              \n"
        "s32i a6, %2, 0        \n" // update rle_buff_p value

        :
        : "a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p) : "a4", "a5", "a6", "a7", "a8", "a9", "a10", "a11", "memory");

    // clockb = xthal_get_ccount();

    // if(time_debug_indice_rle_p < time_debug_indice_lenght)
    // time_debug_indice_rle[time_debug_indice_rle_p++]=(clockb);
    // time_debug_indice_rle[time_debug_indice_rle_p++]=(clockb-clocka);

    // Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
    // Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
    // Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );

    /*
        Serial_Debug_Port.printf("RLE Block Output:\r\n");
        for(int i=0; i < sample_size/40 ; i++ )
          Serial_Debug_Port.printf("0x%X, ", rle_buff[i]);
        Serial_Debug_Port.println();
        */
}
