#ifndef _MROS_TRANSPORT_BASE_
#define _MROS_TRANSPORT_BASE_

#include <iostream>
#include <mros/macros.h>

namespace mros {
class MROS_DllAPI TransportBase
{
public:
  virtual bool init(std::string portName) = 0;

  virtual int read(uint8_t* data, int length) = 0;

  virtual bool write(uint8_t* data, int length) = 0;

  virtual bool connected() = 0;

  virtual void close() = 0;
};
}
#endif //_MROS_TRANSPORT_BASE_

