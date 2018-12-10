#include "v4l2/V4L2FrameProcessor.h"
#include <assert.h>
#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <string.h> // strerrno
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#define V4L2_DEVICE "/dev/video0"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

static int xioctl(int fh, unsigned long int request, void *arg)
{
  int r;

  do
  {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);

  return r;
}

/*****
 * Taken from libv4l2 (in v4l-utils)
 *
 * (C) 2008 Hans de Goede <hdegoede@redhat.com>
 *
 * Released under LGPL
 */
#define CLIP(color) (unsigned char)(((color) > 0xFF) ? 0xff : (((color) < 0) ? 0 : (color)))

static void v4lconvert_yuyv_to_rgb24(const unsigned char *src,
                                     unsigned char *dest,
                                     int width, int height,
                                     int stride)
{
  int j;

  while (--height >= 0) {
    for (j = 0; j + 1 < width; j += 2) {
      int u = src[1];
      int v = src[3];
      int u1 = (((u - 128) << 7) +  (u - 128)) >> 6;
      int rg = (((u - 128) << 1) +  (u - 128) +
                ((v - 128) << 2) + ((v - 128) << 1)) >> 3;
      int v1 = (((v - 128) << 1) +  (v - 128)) >> 1;

      *dest++ = CLIP(src[0] + v1);
      *dest++ = CLIP(src[0] - rg);
      *dest++ = CLIP(src[0] + u1);

      *dest++ = CLIP(src[2] + v1);
      *dest++ = CLIP(src[2] - rg);
      *dest++ = CLIP(src[2] + u1);
      src += 4;
    }
    src += stride - (width * 2);
  }
}

namespace SL
{
namespace Screen_Capture
{
V4L2FrameProcessor::V4L2FrameProcessor()
  : m_fd(-1)
  , m_numBuffers(0)
  , m_buffers(nullptr)
{
  m_device = V4L2_DEVICE;
}

V4L2FrameProcessor::~V4L2FrameProcessor()
{
  if (m_fd != -1)
  {
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(m_fd, VIDIOC_STREAMOFF, &type))
      throw std::runtime_error("VIDIOC_STREAMOFF");

    /* */

    unsigned int i;

    for (i = 0; i < m_numBuffers; ++i)
      if (-1 == munmap(m_buffers[i].data, m_buffers[i].size))
        throw std::runtime_error("munmap");

    free(m_buffers);

    /* */

    if (-1 == close(m_fd))
      throw std::runtime_error("close");

    m_fd = -1;
  }
}

void V4L2FrameProcessor::setup_capture()
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;

  if (-1 == xioctl(m_fd, VIDIOC_QUERYCAP, &cap))
  {
    if (EINVAL == errno)
      throw std::runtime_error(m_device + " is no V4L2 device");
    else
      throw std::runtime_error("VIDIOC_QUERYCAP");
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    throw std::runtime_error(m_device + " is no video capture device");

  if (!(cap.capabilities & V4L2_CAP_STREAMING))
    throw std::runtime_error(m_device + " does not support streaming i/o");

  /* Select video input, video standard and tune here. */

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == xioctl(m_fd, VIDIOC_CROPCAP, &cropcap))
  {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == xioctl(m_fd, VIDIOC_S_CROP, &crop))
    {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  CLEAR(fmt);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (false /*force_format*/)
  {
    fmt.fmt.pix.width       = 1280;
    fmt.fmt.pix.height      = 720;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_ARGB32;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (-1 == xioctl(m_fd, VIDIOC_S_FMT, &fmt))
      throw std::runtime_error("VIDIOC_S_FMT");

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_ARGB32)
      // note that libv4l2 (look for 'v4l-utils') provides helpers
      // to manage conversions
      throw std::runtime_error("V4L2 device does not support ARGB32 format. Support for more format need to be added!");
  }
  else
  {
    /* Preserve original settings as set by v4l2-ctl for example */
    if (-1 == xioctl(m_fd, VIDIOC_G_FMT, &fmt))
      throw std::runtime_error("VIDIOC_G_FMT");

    assert(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_ARGB32);
  }

  m_xres = fmt.fmt.pix.width;
  m_yres = fmt.fmt.pix.height;
  m_stride = fmt.fmt.pix.bytesperline;
}

DUPL_RETURN V4L2FrameProcessor::Init(std::shared_ptr<Thread_Data> data, const Window& selectedwindow)
{
  return DUPL_RETURN::DUPL_RETURN_ERROR_EXPECTED;
}

DUPL_RETURN V4L2FrameProcessor::ProcessFrame(Window& selectedwindow)
{
  return DUPL_RETURN::DUPL_RETURN_ERROR_EXPECTED;
}

DUPL_RETURN V4L2FrameProcessor::Init(std::shared_ptr<Thread_Data> data, Monitor& monitor)
{
  auto ret = DUPL_RETURN::DUPL_RETURN_SUCCESS;
  Data = data;

  struct stat st;

  if (-1 == stat(m_device.c_str(), &st))
    throw std::runtime_error(m_device + ": cannot identify! " + std::to_string(errno) +  ": " + strerror(errno));

  if (!S_ISCHR(st.st_mode))
    throw std::runtime_error(m_device + " is no device");

  m_fd = open(m_device.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == m_fd)
    throw std::runtime_error(m_device + ": cannot open! " + std::to_string(errno) + ": " + strerror(errno));

  setup_capture();

  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(m_fd, VIDIOC_REQBUFS, &req))
  {
    if (EINVAL == errno)
      throw std::runtime_error(m_device + " does not support memory mapping");
    else
      throw std::runtime_error("VIDIOC_REQBUFS");
  }

  if (req.count < 2)
    throw std::runtime_error(std::string("Insufficient buffer memory on ") + m_device);

  m_buffers = static_cast<buffer*>(calloc(req.count, sizeof(*m_buffers)));
  if (!m_buffers)
    throw std::runtime_error("Out of memory");

  for (m_numBuffers = 0; m_numBuffers < req.count; ++m_numBuffers)
  {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = m_numBuffers;

    if (-1 == xioctl(m_fd, VIDIOC_QUERYBUF, &buf))
      throw std::runtime_error("VIDIOC_QUERYBUF");

    m_buffers[m_numBuffers].size = buf.length;
    m_buffers[m_numBuffers].data =
        mmap(nullptr /* start anywhere */,
             buf.length,
             PROT_READ | PROT_WRITE /* required */,
             MAP_SHARED /* recommended */,
             m_fd, buf.m.offset);

    if (MAP_FAILED == m_buffers[m_numBuffers].data)
      throw std::runtime_error("mmap");
  }

  /* */

  for (unsigned int i = 0; i < m_numBuffers; ++i)
  {
    struct v4l2_buffer buf;

    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (-1 == xioctl(m_fd, VIDIOC_QBUF, &buf))
      throw std::runtime_error("VIDIOC_QBUF");
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(m_fd, VIDIOC_STREAMON, &type))
    throw std::runtime_error("VIDIOC_STREAMON");

  return ret;
}

DUPL_RETURN V4L2FrameProcessor::ProcessFrame(const Monitor& curentmonitorinfo)
{
  auto Ret = DUPL_RETURN_SUCCESS;

  struct v4l2_buffer buf;
  CLEAR(buf);

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(m_fd, VIDIOC_DQBUF, &buf)) {
    switch (errno) {
      case EAGAIN:
        return DUPL_RETURN_ERROR_EXPECTED;

      case EIO:
        /* Could ignore EIO, see spec. */

        /* fall through */

      default:
        throw std::runtime_error("VIDIOC_DQBUF");
    }
  }

  assert(buf.index < m_numBuffers);

//  v4lconvert_yuyv_to_rgb24((unsigned char *) buffers[buf.index].data,
//      rgb_frame.data,
//      xres,
//      yres,
//      stride);

  ProcessCapture(Data->ScreenCaptureData, *this, curentmonitorinfo, (unsigned char *) m_buffers[buf.index].data, m_stride);

  if (-1 == xioctl(m_fd, VIDIOC_QBUF, &buf))
    throw std::runtime_error("VIDIOC_QBUF");

  return Ret;
}
}
}