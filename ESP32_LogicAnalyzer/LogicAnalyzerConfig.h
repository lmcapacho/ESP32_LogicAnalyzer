#ifndef LOGIC_ANALYZER_CONFIG_H
#define LOGIC_ANALYZER_CONFIG_H

#define TAG "esp32la"

// Uncomment to enable debug output (uses Serial_Debug_Port)
//#define _DEBUG_MODE_

// ========================================
// OLS Port and Serial Debug Configuration
// ========================================

#ifdef _DEBUG_MODE_
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    // ESP32-S3 DevKitC-1 dual-USB split:
    // - OLS_Port on native USB (USBSerial) for PulseView/SUMP
    // - Serial_Debug_Port on UART0/USB bridge for logs
    #define OLS_Port USBSerial
    #define OLS_Port_Baud 3000000

    #define Serial_Debug_Port Serial
    #define Serial_Debug_Port_Baud 921600
  #else
    #define OLS_Port Serial2
    #define OLS_Port_Baud 3000000

    #define Serial_Debug_Port Serial
    #define Serial_Debug_Port_Baud 921600
  #endif
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
// Board Pin Assignments
// ==============================

#include "LogicAnalyzerBoardConfig.h"

#endif // LOGIC_ANALYZER_CONFIG_H
