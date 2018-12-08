#include "ScreenCapture.h"
#include "internal/ThreadManager.h"
#include "demo/DemoFrameProcessor.h"

namespace SL{
    namespace Screen_Capture{	
        void RunCaptureMouse(std::shared_ptr<Thread_Data> data) {
        }
        void RunCaptureMonitor(std::shared_ptr<Thread_Data> data, Monitor monitor){
            TryCaptureMonitor<DemoFrameProcessor>(data, monitor);
        }
        void RunCaptureWindow(std::shared_ptr<Thread_Data> data, Window window){
            TryCaptureWindow<DemoFrameProcessor>(data, window);
        }
    }
}

  
    
