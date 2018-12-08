#pragma once
#include "internal/SCCommon.h"
#include <memory>

namespace SL {
    namespace Screen_Capture {
   
        class DemoFrameProcessor: public BaseFrameProcessor {
            
            Monitor SelectedMonitor;
            
        public:
            DemoFrameProcessor();
            ~DemoFrameProcessor();
			
            void Pause() {}
            void Resume() {}
            DUPL_RETURN Init(std::shared_ptr<Thread_Data> data, Monitor& monitor);
            DUPL_RETURN ProcessFrame(const Monitor& currentmonitorinfo);
            DUPL_RETURN Init(std::shared_ptr<Thread_Data> data, const Window& selectedwindow);
            DUPL_RETURN ProcessFrame(Window& selectedwindow);
        };
    }
}