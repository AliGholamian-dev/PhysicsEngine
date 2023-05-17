#include "Settings.h"

namespace Settings {
    namespace Debug      {Setting setting;}
    namespace Exception  {Setting setting;}
}

Settings::Debug::Setting::Setting() :
    showDebugMessages(true),
    debugModes(DebugMode::inConsole)
{}

Settings::Exception::Setting::Setting() :
        showExceptionMessage(true)
{}
