#pragma once

#include "../capture_backend_bridge.h"

namespace capture_backend_esp32 {

bool init(const capture_backend_bridge::Config &cfg);
void start();
void stop();

} // namespace capture_backend_esp32
