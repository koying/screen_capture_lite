#include "ScreenCapture.h"
#include "internal/SCCommon.h"

namespace SL
{
namespace Screen_Capture
{

    std::vector<Monitor> GetMonitors()
    {
      std::vector<Monitor> ret;

      auto name = std::string("Display ") + std::to_string(0);
      ret.push_back(CreateMonitor(
                0, 0, 1080, 1920, 0, 0, name, 1.0f));
        return ret;
    }
}
}
