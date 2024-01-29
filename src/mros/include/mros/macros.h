#ifndef MROS_MACROS_H_INCLUDED
#define MROS_MACROS_H_INCLUDED

#define MROS_LOG_TOPIC ("/mrosout")
#define MROS_DISCOVERY ("/mrosdiscovery")
#define MROS_STATISTICS ("/mrosstatistics")
#define MROS_CLOCK_TOPIC ("/clock")
#define MROS_DIAGNOSTICS_TOPIC ("/diagnostics")
#define MROS_DIAGNOSTICS_VALUE_TOPIC ("/diagnostics_value")
#define MROS_CALLERID_SIZE (32)

#if defined(_MSC_VER)
  #define MROS_DllAPI __declspec(dllexport)
#elif __GNUC__ >= 4
  #define MROS_DllAPI __attribute__ ((visibility("default")))
#else
  #define MROS_DllAPI
#endif

#ifdef _MSC_VER
  #pragma warning(disable: 4251)
  #pragma warning(disable: 4275)
#endif

#endif