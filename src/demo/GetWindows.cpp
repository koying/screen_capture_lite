#include "ScreenCapture.h"
#include "internal/SCCommon.h"
#include <algorithm>
#include <string>
#include <vector>
#include <iostream>


namespace SL
{
namespace Screen_Capture
{
    std::vector<Window> GetWindows()
    {
        std::vector<Window> ret;

        Window w = {};
        w.Handle = 0;
        w.Position = Point{ 0, 0 };
        w.Size = Point{ 1920, 1080 };
        strcpy(w.Name, "DemoW");

        return ret;
    }
}
}
