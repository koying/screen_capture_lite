#include "demo/DemoFrameProcessor.h"
#include <assert.h>
#include <vector>

#include "demo_img.c"

namespace SL
{
namespace Screen_Capture
{
    DemoFrameProcessor::DemoFrameProcessor()
    {
    }

    DemoFrameProcessor::~DemoFrameProcessor()
    {
    }
    
    DUPL_RETURN DemoFrameProcessor::Init(std::shared_ptr<Thread_Data> data, const Window& selectedwindow){
        
        auto ret = DUPL_RETURN::DUPL_RETURN_SUCCESS;
        Data = data;
        return ret;
    }
    DUPL_RETURN DemoFrameProcessor::Init(std::shared_ptr<Thread_Data> data, Monitor& monitor)
    {
        auto ret = DUPL_RETURN::DUPL_RETURN_SUCCESS;
        Data = data;
        return ret;
    }
 
    DUPL_RETURN DemoFrameProcessor::ProcessFrame(const Monitor& curentmonitorinfo)
    {        
        auto Ret = DUPL_RETURN_SUCCESS;
        ProcessCapture(Data->ScreenCaptureData, *this, curentmonitorinfo, (unsigned char*)_demo_bmp, 1920 * 4);
        return Ret;
    }
    DUPL_RETURN DemoFrameProcessor::ProcessFrame(Window& selectedwindow){
        
        auto Ret = DUPL_RETURN_SUCCESS; 
        ProcessCapture(Data->WindowCaptureData, *this, selectedwindow, (unsigned char*)_demo_bmp, 1920 * 4);
        return Ret;
    }
}
}