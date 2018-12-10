#pragma once
#include "internal/SCCommon.h"
#include <memory>

namespace SL {
namespace Screen_Capture {

class V4L2FrameProcessor: public BaseFrameProcessor
{
  struct buffer
  {
    void   *data;
    size_t  size;
  };

  Monitor SelectedMonitor;

public:
  V4L2FrameProcessor();
  ~V4L2FrameProcessor();

  void Pause() {}
  void Resume() {}
  DUPL_RETURN Init(std::shared_ptr<Thread_Data> data, Monitor& monitor);
  DUPL_RETURN ProcessFrame(const Monitor& currentmonitorinfo);
  DUPL_RETURN Init(std::shared_ptr<Thread_Data> data, const Window& selectedwindow);
  DUPL_RETURN ProcessFrame(Window& selectedwindow);

private:
  std::string m_device;
  int m_fd;
  uint32_t m_xres;
  uint32_t m_yres;
  uint32_t m_stride;

  void setup_capture();

  unsigned int m_numBuffers;
  struct buffer *m_buffers;

};
}
}