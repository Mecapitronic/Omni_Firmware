#include "Strategy/Action.h"

#ifdef SIMULATOR
extern "C" const char embeddedActionsJson[] asm("_binary_data_actions_json_start");
#endif

// Load simulation fixtures from PlatformIO's embedded text sections.
static bool _mockspiffs_preload_actions = []() {
#ifdef SIMULATOR
    MockSPIFFS.preload("/actions.json", String(embeddedActionsJson));
#endif
    return true;
}();