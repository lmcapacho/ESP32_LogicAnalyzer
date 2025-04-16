# ESP32 LogicAnalyzer

![Status](https://img.shields.io/badge/Status-In%20Development-yellow)
![License](https://img.shields.io/badge/License-GPL%203.0-blue)

A [SUMP](http://dangerousprototypes.com/docs/The_Logic_Sniffer%27s_extended_SUMP_protocol) compatible 16Bit Logic Analyzer for ESP32 MCUs.

<img src="/docs/images/ESP32_LogicAnalyzer_in_PulseView.png" alt="PulseView" style="width: 80%;" />

## Specifications

- Uses **ESP32 I2S DMA** and can capture speeds up to **20 MHz**.
- Supports **8-bit** and **16-bit** operations.
- Maximum **128k** samples (even in 8-bit capturing mode).
- RLE compression supported.
- Default OLS port is **UART0** with a default baud rate of **912600**.
- You can configure UART ports and high-speed baud rates for OLS communication by editing the definitions in LogicAnalyzerConfig.h.

## WARNING

- **For OLS port at UART0**:
  - Set **"Core Debug Level" = None** before compiling the code in Arduino, especially for > 10 MHz capture operations.
- **GPIO23** is used for I2S input clock, and **GPIO22** is used for LEDC clock output. Do not use these pins for I/O or change them and connect them to other unused pins in the code.

## Under Development

- Arduino support to compile and flash ESP32.
- WROOVER modules support **2M** samples but only up to **2 MHz** due to bandwidth limits on PSRAM access.
- Analog input support.

## Quick Start Guide

1. Install [PlatformIO](https://platformio.org/) in VS Code.
2. Open the project folder in PlatformIO.
3. Build the project and flash it to your ESP32.

### PulseView

Open PulseView with:
```bash
pulseview -D -d ols:conn=/dev/ttyUSB0::serialcomm=921600/8n1
```
Or from the GUI, connect the device and select **Openbench Logic Sniffer & SUMP compatibles (ols)**.

![PulseView](/docs/images/connect_to_device_pulseview.png)

RLE can be enabled in Device configuration dialog:

![RLE](/docs/images/RLE_enable.png)

For more details, see the [PulseView documentation](https://sigrok.org/wiki/PulseView).

## Connections

| Pin      | Connection                              |
|----------|-----------------------------------------|
| clk_in   | GPIO23 (connect to **clk_out**)         |
| clk_out  | GPIO22 (connect to **clk_in**)          |
| Ground   | Connect device GND to ESP32 GND         |
| Channels | CH15 - CH0 (connect your signals here)  |

<img src="/docs/images/esp32_la_conn.png" alt="ESP32LA Connection" style="width: 50%;" />

## Attributions

This project is a fork of [ESP32_LogicAnalyzer](https://github.com/EUA/ESP32_LogicAnalyzer), created by [Erdem U. Altinyurt](https://github.com/EUA).

This project uses code from:
- [esp32-cam-demo](https://github.com/igrr/esp32-cam-demo) for I2S DMA.
- [Arduino Logic Analyzer](https://github.com/gillham/logic_analyzer) as a SUMP protocol "template".

## License

This project is licensed under the **GPL 3.0 License**. See the [LICENSE](LICENSE) file for details.
