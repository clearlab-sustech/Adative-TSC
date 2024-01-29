#ifndef _MROS_LOG_H_
#define _MROS_LOG_H_

#include <mros/macros.h>
#include "mros/os/time.h"
#include "mros/mros_msgs/Log.h"

namespace mros
{
MROS_DllAPI void mtrace(int level, char const *file, int line, char const *func, bool screen, const char *chfr, ...);

#define mros_log_debug(format, ...) mros::mtrace(1, __FILE__, __LINE__, __func__, false, format, ##__VA_ARGS__)
#define mros_log_info(format, ...) mros::mtrace(2, __FILE__, __LINE__, __func__, false, format, ##__VA_ARGS__)
#define mros_log_warn(format, ...) mros::mtrace(4, __FILE__, __LINE__, __func__, false, format, ##__VA_ARGS__)
#define mros_log_error(format, ...) mros::mtrace(8, __FILE__, __LINE__, __func__, false, format, ##__VA_ARGS__)
#define mros_log_fatal(format, ...) mros::mtrace(16, __FILE__, __LINE__, __func__, false, format, ##__VA_ARGS__)

#define MROS_LOG_ONCE_TIME_THROTTLE  1
#define mros_log_once_debug(format, ...) { \
  static double __once_time = -1.0f; \
  double __once_tem_sec =(mros::Time::WallTime().toSec() - __once_time); \
  if((__once_tem_sec > MROS_LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = mros::Time::WallTime().toSec(); \
    mros::mtrace(1,__FILE__, __LINE__, __func__, false, format, ##__VA_ARGS__); \
  } \
}
#define mros_log_once_info(format, ...) { \
  static double __once_time = -1.0f; \
  double __once_tem_sec =(mros::Time::WallTime().toSec() - __once_time); \
  if((__once_tem_sec > MROS_LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = mros::Time::WallTime().toSec(); \
    mros::mtrace(2,__FILE__, __LINE__, __func__, false, format, ##__VA_ARGS__); \
  } \
}
#define mros_log_once_warn(format, ...) { \
  static double __once_time = -1.0f; \
  double __once_tem_sec =(mros::Time::WallTime().toSec() - __once_time); \
  if((__once_tem_sec > MROS_LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = mros::Time::WallTime().toSec(); \
    mros::mtrace(4,__FILE__, __LINE__, __func__, false, format, ##__VA_ARGS__); \
  } \
}
#define mros_log_once_error(format, ...) { \
  static double __once_time = -1.0f; \
  double __once_tem_sec =(mros::Time::WallTime().toSec() - __once_time); \
  if((__once_tem_sec > MROS_LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = mros::Time::WallTime().toSec(); \
    mros::mtrace(8,__FILE__, __LINE__, __func__, false, format, ##__VA_ARGS__); \
  } \
}
#define mros_log_once_fatal(format, ...) { \
  static double __once_time = -1.0f; \
  double __once_tem_sec =(mros::Time::WallTime().toSec() - __once_time); \
  if((__once_tem_sec > MROS_LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = mros::Time::WallTime().toSec(); \
    mros::mtrace(16,__FILE__, __LINE__, __func__, false, format, ##__VA_ARGS__); \
  } \
}

//////////////////////////////////////////////////////////////////////////////////////////////

#define mros_log_debug_screen(format, ...) mros::mtrace(1, __FILE__, __LINE__, __func__, true, format, ##__VA_ARGS__)
#define mros_log_info_screen(format, ...) mros::mtrace(2, __FILE__, __LINE__, __func__, true, format, ##__VA_ARGS__)
#define mros_log_warn_screen(format, ...) mros::mtrace(4, __FILE__, __LINE__, __func__, true, format, ##__VA_ARGS__)
#define mros_log_error_screen(format, ...) mros::mtrace(8, __FILE__, __LINE__, __func__, true, format, ##__VA_ARGS__)
#define mros_log_fatal_screen(format, ...) mros::mtrace(16, __FILE__, __LINE__, __func__, true, format, ##__VA_ARGS__)

#define MROS_LOG_ONCE_TIME_THROTTLE  1
#define mros_log_once_debug_screen(format, ...) { \
  static double __once_time = -1.0f; \
  double __once_tem_sec =(mros::Time::WallTime().toSec() - __once_time); \
  if((__once_tem_sec > MROS_LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = mros::Time::WallTime().toSec(); \
    mros::mtrace(1,__FILE__, __LINE__, __func__, true, format, ##__VA_ARGS__); \
  } \
}
#define mros_log_once_info_screen(format, ...) { \
  static double __once_time = -1.0f; \
  double __once_tem_sec =(mros::Time::WallTime().toSec() - __once_time); \
  if((__once_tem_sec > MROS_LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = mros::Time::WallTime().toSec(); \
    mros::mtrace(2,__FILE__, __LINE__, __func__, true, format, ##__VA_ARGS__); \
  } \
}
#define mros_log_once_warn_screen(format, ...) { \
  static double __once_time = -1.0f; \
  double __once_tem_sec =(mros::Time::WallTime().toSec() - __once_time); \
  if((__once_tem_sec > MROS_LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = mros::Time::WallTime().toSec(); \
    mros::mtrace(4,__FILE__, __LINE__, __func__, true, format, ##__VA_ARGS__); \
  } \
}
#define mros_log_once_error_screen(format, ...) { \
  static double __once_time = -1.0f; \
  double __once_tem_sec =(mros::Time::WallTime().toSec() - __once_time); \
  if((__once_tem_sec > MROS_LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = mros::Time::WallTime().toSec(); \
    mros::mtrace(8,__FILE__, __LINE__, __func__, true, format, ##__VA_ARGS__); \
  } \
}
#define mros_log_once_fatal_screen(format, ...) { \
  static double __once_time = -1.0f; \
  double __once_tem_sec =(mros::Time::WallTime().toSec() - __once_time); \
  if((__once_tem_sec > MROS_LOG_ONCE_TIME_THROTTLE) || (__once_tem_sec < 0.0f)) { \
    __once_time = mros::Time::WallTime().toSec(); \
    mros::mtrace(16,__FILE__, __LINE__, __func__, true, format, ##__VA_ARGS__); \
  } \
}

MROS_DllAPI void set_mros_log_level(int level);

}  // namespace mros

#endif


