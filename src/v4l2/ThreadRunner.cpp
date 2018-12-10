#include "ScreenCapture.h"
#include "internal/ThreadManager.h"
#include "v4l2/V4L2FrameProcessor.h"

namespace SL{
    namespace Screen_Capture{	
        void RunCaptureMouse(std::shared_ptr<Thread_Data> data) {
        }
        void RunCaptureMonitor(std::shared_ptr<Thread_Data> data, Monitor monitor){
            TryCaptureMonitor<V4L2FrameProcessor>(data, monitor);
        }
        void RunCaptureWindow(std::shared_ptr<Thread_Data> data, Window window){
            TryCaptureWindow<V4L2FrameProcessor>(data, window);
        }
    }
}

  
    
