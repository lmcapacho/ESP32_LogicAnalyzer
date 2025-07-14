#ifndef LOGIC_ANALYZER_CONFIG_H
#define LOGIC_ANALYZER_CONFIG_H

#define TAG "esp32la"

// Uncomment to enable debug output (uses Serial_Debug_Port)
//#define _DEBUG_MODE_

// ========================================
// OLS Port and Serial Debug Configuration
// ========================================

#ifdef _DEBUG_MODE_
  #define OLS_Port Serial2
  #define OLS_Port_Baud 3000000

  #define Serial_Debug_Port Serial
  #define Serial_Debug_Port_Baud 921600
#else
  #define OLS_Port Serial   
  #define OLS_Port_Baud 921600
#endif

// ==========================
// Compression configuration
// ==========================

/*
 * ALLOW_ZERO_RLE:
 * ---------------
 * ALLOW_ZERO_RLE 1 is Fast mode.
 *  Add RLE Count 0 to RLE stack for non repeated values and postpone the RLE processing so faster.
 *      8Bit Mode : ~28.4k clock per 4k block, captures 3000us while inspecting ~10Mhz clock at 20Mhz mode
 *      16Bit Mode : ~22.3k clock per 4k block, captures 1500us while inspecting ~10Mhz clock at 20Mhz mode
 * 
 * ALLOW_ZERO_RLE 0 is Slow mode.
 *  Just RAW RLE buffer. It doesn't add 0 count values for non-repeated RLE values and process flags on the fly, 
 *  so little slow but efficient.
 *      8Bit Mode : ~34.7k clock per 4k block, captures 4700us while inspecting ~10Mhz clock at 20Mhz mode
 *      16Bit Mode : ~30.3k clock per 4k block, captures 2400us while inspecting ~10Mhz clock at 20Mhz mode
 */
#define ALLOW_ZERO_RLE 0

// ==============================
// Capture Settings
// ==============================

#define CAPTURE_SIZE      128000   // total number of samples
#define RLE_BUFFER_SIZE    96000   // buffer size for compressed data

// ==============================
// Pin Assignments
// ==============================

#define LED_PIN 2  // LED lights up while capturing

// Channel input pins
#define CH0_PIN   21
#define CH1_PIN   19
#define CH2_PIN   18
#define CH3_PIN   5
#define CH4_PIN   4
#define CH5_PIN   15
#define CH6_PIN   13
#define CH7_PIN   12
#define CH8_PIN   14
#define CH9_PIN   27
#define CH10_PIN  26
#define CH11_PIN  25
#define CH12_PIN  33
#define CH13_PIN  32
#define CH14_PIN  35
#define CH15_PIN  34

// Clock and sync lines
#define CLK_OUT 22
#define CLK_IN  23

#endif // LOGIC_ANALYZER_CONFIG_H
