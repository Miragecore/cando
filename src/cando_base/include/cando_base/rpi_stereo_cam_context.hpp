#ifndef RPI_STEREO_CAM_CONTEXT_HPP
#define RPI_STEREO_CAM_CONTEXT_HPP

#include <cmath>
#include <string>

#include "ros2_shared/context_macros.hpp"

namespace rpi_stereo_cam 
{

#define RPI_STEREO_CAM_ALL_PARAMS \
  /* CXT_MACRO_MEMBER(width, int, 0)                                  Device width */ \
  /* CXT_MACRO_MEMBER(height, int, 0)                                 Device height */ \
  \
  CXT_MACRO_MEMBER(lcamera_info_path, std::string, "left.yaml")     /* leftCamera info path */ \
  CXT_MACRO_MEMBER(lcamera_frame_id, std::string, "lcamera_frame")  /* leftCamera frame id */ \
  CXT_MACRO_MEMBER(rcamera_info_path, std::string, "right.yaml")     /* RightCamera info path */ \
  CXT_MACRO_MEMBER(rcamera_frame_id, std::string, "rcamera_frame")  /* RightCamera frame id */ \
/* End of list */

  struct RpiStereoCameraContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    CXT_MACRO_DEFINE_MEMBERS(RPI_STEREO_CAM_ALL_PARAMS)
  };

} // namespace rpi_stereo_cam

#endif // RPI_STEREO_CAM_CONTEXT_HPP
