
  /*************************************************************************
 * 
 *  ESP32 Logic Analyzer
 *  Copyright (C) 2025 lmcapacho (Luis Miguel Capacho V.)  
 *  Copyright (C) 2020 Erdem U. Altinyurt
 *    
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *************************************************************************
 *
 *  This project steals some code from 
 *  https://github.com/igrr/esp32-cam-demo for I2S DMA
 *  and
 *  https://github.com/gillham/logic_analyzer/issues for SUMP protocol as template.
 * 
 */ 
#include "LogicAnalyzer.h"

i2s_parallel_buffer_desc_t bufdesc;
LogicAnalyzer esp32la;

void setup(void) {
  esp32la.begin(&bufdesc);
}

void loop()
{
  vTaskDelay(1); //To avoid WDT
  
  if (OLS_Port.available() > 0) {
    int cmdByte = OLS_Port.read();
    esp32la.handleCommand(cmdByte);    
  }
}

void IRAM_ATTR i2s_wrapper(void *arg)
{
  esp32la.i2s_isr(arg);
}
