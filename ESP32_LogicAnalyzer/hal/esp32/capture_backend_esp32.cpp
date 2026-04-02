#include "capture_backend_esp32.h"

namespace capture_backend_esp32 {

bool init(const capture_backend_bridge::Config &cfg)
{
    (void)cfg;
    return true;
}

void start() {}

void stop() {}

} // namespace capture_backend_esp32
