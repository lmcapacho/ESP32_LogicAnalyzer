#ifndef LOGIC_ANALYZER_BOARD_CONFIG_H
#define LOGIC_ANALYZER_BOARD_CONFIG_H

// Board-profile selection order:
// 1. Explicit profile macro from build flags
// 2. Target-based fallback
// 3. Optional local override file (ignored by git)

#if defined(LOGIC_ANALYZER_BOARD_PROFILE_ESP32DEV)
#include "board_profiles/LogicAnalyzerBoardProfile_ESP32Dev.h"
#elif defined(LOGIC_ANALYZER_BOARD_PROFILE_ESP32S3_DEVKITC_1)
#include "board_profiles/LogicAnalyzerBoardProfile_ESP32S3DevKitC1.h"
#elif defined(LOGIC_ANALYZER_BOARD_PROFILE_ESP32S3_GENERIC)
#include "board_profiles/LogicAnalyzerBoardProfile_ESP32S3Generic.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include "board_profiles/LogicAnalyzerBoardProfile_ESP32S3Generic.h"
#else
#include "board_profiles/LogicAnalyzerBoardProfile_ESP32Dev.h"
#endif

#if __has_include("LogicAnalyzerBoardConfig.local.h")
#include "LogicAnalyzerBoardConfig.local.h"
#endif

#endif // LOGIC_ANALYZER_BOARD_CONFIG_H
