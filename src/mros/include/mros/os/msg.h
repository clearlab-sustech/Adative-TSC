#ifndef _MROS_MSG_H_
#define _MROS_MSG_H_
#include <stdint.h>
#include <stddef.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <vector>
#include <stdexcept>
#include <mros/macros.h>
#include "mros/json/reader.h"
#include "mros/json/writer.h"
#include "mros/json/elements.h"

namespace mros
{
class Msg;
typedef std::shared_ptr<mros::Msg> MsgPtr;
typedef std::shared_ptr<mros::Msg const> MsgConstPtr;

/* Base Message Type */
class Msg
{
public:
  std::string callerid;

public:
  virtual int serialize(unsigned char *outbuffer, int count) const = 0;
  virtual int deserialize(unsigned char *data, int count) = 0;
  virtual int serializedLength() const = 0;
  virtual const std::string getType() const = 0;
  virtual const std::string getMD5() const = 0;
  virtual const std::string getDefinition() const = 0;
  virtual const std::string echo() const = 0;
  virtual const Msg* getHeader() const { return nullptr; }
  virtual void parse(Msg* msg, const std::string& json) {  }

  union Fp32 {
    uint32_t u;
    float f;
  };
  
  static uint16_t float32CovUint16(float value) {
    const Fp32 f32infty = { 255U << 23 };
    const Fp32 f16infty = { 31U << 23 };
    const Fp32 magic = { 15U << 23 };
    const uint32_t sign_mask = 0x80000000U;
    const uint32_t round_mask = ~0xFFFU;

    Fp32 in;
    uint16_t out;

    in.f = value;

    uint32_t sign = in.u & sign_mask;
    in.u ^= sign;

    if (in.u >= f32infty.u) { /* Inf or NaN (all exponent bits set) */
      /* NaN->sNaN and Inf->Inf */
      out = (in.u > f32infty.u) ? 0x7FFFU : 0x7C00U;
    } else { /* (De)normalized number or zero */
      in.u &= round_mask;
      in.f *= magic.f;
      in.u -= round_mask;
      if (in.u > f16infty.u) {
        in.u = f16infty.u; /* Clamp to signed infinity if overflowed */
      }
      out = uint16_t(in.u >> 13); /* Take the bits! */
    }
    out = uint16_t(out | (sign >> 16));
    return out;
  }

  static float uint16CovFloat32(uint16_t value) {
    const Fp32 magic = { (254U - 15U) << 23 };
    const Fp32 was_infnan = { (127U + 16U) << 23 };
    Fp32 out;

    out.u = (value & 0x7FFFU) << 13;   /* exponent/mantissa bits */
    out.f *= magic.f;                  /* exponent adjust */
    if (out.f >= was_infnan.f) {        /* make sure Inf/NaN survive */
      out.u |= 255U << 23;
    }
    out.u |= (value & 0x8000U) << 16;  /* sign bit */
    return out.f;
  }
  
  /**
   * @brief This tricky function handles promoting a 32bit float to a 64bit
   *        double, so that AVR can publish messages containing float64
   *        fields, despite AVV having no native support for double.
   *
   * @param[out] outbuffer pointer for buffer to serialize to.
   * @param[in] f value to serialize.
   *
   * @return number of bytes to advance the buffer pointer.
   *
   */
  static int serializeAvrFloat64(unsigned char* outbuffer, const float f) {
    const int32_t* val = (int32_t*) &f;
    int32_t exp = ((*val >> 23) & 255);
    if (exp != 0) {
      exp += 1023 - 127;
    }

    int32_t sig = *val;
    *(outbuffer++) = 0;
    *(outbuffer++) = 0;
    *(outbuffer++) = 0;
    *(outbuffer++) = (sig << 5) & 0xff;
    *(outbuffer++) = (sig >> 3) & 0xff;
    *(outbuffer++) = (sig >> 11) & 0xff;
    *(outbuffer++) = ((exp << 4) & 0xF0) | ((sig >> 19) & 0x0F);
    *(outbuffer++) = (exp >> 4) & 0x7F;

    // Mark negative bit as necessary.
    if (f < 0) {
      *(outbuffer - 1) |= 0x80;
    }

    return 8;
  }

  /**
   * @brief This tricky function handles demoting a 64bit double to a
   *        32bit float, so that AVR can understand messages containing
   *        float64 fields, despite AVR having no native support for double.
   *
   * @param[in] inbuffer pointer for buffer to deserialize from.
   * @param[out] f pointer to place the deserialized value in.
   *
   * @return number of bytes to advance the buffer pointer.
   */
  static int deserializeAvrFloat64(const unsigned char* inbuffer, float* f) {
    uint32_t* val = (uint32_t*)f;
    inbuffer += 3;

    // Copy truncated mantissa.
    *val = ((uint32_t)(*(inbuffer++)) >> 5 & 0x07);
    *val |= ((uint32_t)(*(inbuffer++)) & 0xff) << 3;
    *val |= ((uint32_t)(*(inbuffer++)) & 0xff) << 11;
    *val |= ((uint32_t)(*inbuffer) & 0x0f) << 19;

    // Copy truncated exponent.
    uint32_t exp = ((uint32_t)(*(inbuffer++)) & 0xf0) >> 4;
    exp |= ((uint32_t)(*inbuffer) & 0x7f) << 4;
    if (exp != 0) {
      *val |= ((exp) - 1023 + 127) << 23;
    }

    // Copy negative sign.
    *val |= ((uint32_t)(*(inbuffer++)) & 0x80) << 24;

    return 8;
  }

  // Copy data from variable into a byte array
  template<typename A, typename V>
  static void varToArr(A arr, const V var) {
    for (size_t i = 0; i < sizeof(V); i++)
      arr[i] = (var >> (8 * i));
  }

  // Copy data from a byte array into variable
  template<typename V, typename A>
  static void arrToVar(V& var, const A arr) {
    var = 0;
    for (size_t i = 0; i < sizeof(V); i++)
      var |= (arr[i] << (8 * i));
  }

  static std::string matchString1(const std::string& raw) {
    uint16_t unicode_multi = 0;
    std::string string;

    for (int32_t i =0; i < raw.size(); ) {
      char c = raw[i];

      // escape?
      if (c == '\\' && (i + 1) < raw.size()) { // shouldn't have reached the end yet
         i += 1;
         c = raw[i];
         if (unicode_multi != 0 && c != 'u') {
          std::string message = std::string("Missing unicode low surrogate in string: ") + c;
          throw std::runtime_error(message);
         }
         switch (c) {
            case '/':      string.push_back('/');     break;
            case '"':      string.push_back('"');     break;
            case '\\':     string.push_back('\\');    break;
            case 'b':      string.push_back('\b');    break;
            case 'f':      string.push_back('\f');    break;
            case 'n':      string.push_back('\n');    break;
            case 'r':      string.push_back('\r');    break;
            case 't':      string.push_back('\t');    break;
            case 'u': { // convert unicode to UTF-8
              int x = 0, j;

              // next four characters should be hex
              for (j = 0; j < 4; ++j) {
                i += 1;
                if (i < raw.size()) {
                  c = raw[i];
                  if (c >= '0' && c <= '9') {
                    x = (x << 4) | (c - '0');
                  } else if (c >= 'a' && c <= 'f') {
                    x = (x << 4) | (c - 'a' + 10);
                  } else if (c >= 'A' && c <= 'F') {
                    x = (x << 4) | (c - 'A' + 10);
                  } else {
                    std::string message = std::string("Unrecognized hexadecimal character found in string: ") + c;
                    throw std::runtime_error(message);
                  }
                } else {
                  std::string message = std::string("Overflow raw string");
                  throw std::runtime_error(message);
                }
              }

              // trivial case
              if (x < 0x80) {
                string.push_back(x);
                break;
              }

              // encode surrogate pair
              if (unicode_multi) {
                if (!(((x) & 0xfc00) == 0xdc00)) {
                  std::string message = std::string("Missing unicode low surrogate in string: ") + c; \
                  throw std::runtime_error(message);
                }

                x = 0x10000 + ((unicode_multi & 0x3ff) << 10) + (x & 0x3ff);
                string.push_back((char) ((x >> 18) | 0xf0));
                string.push_back((char) (((x >> 12) & 0x3f) | 0x80));
                string.push_back((char) (((x >> 6) & 0x3f) | 0x80));
                string.push_back((char) ((x & 0x3f) | 0x80));
                unicode_multi = 0;
                break;
              }

              if ((((x) & 0xfc00) == 0xd800)) {
                unicode_multi = x;
                break;
              }

              if ((((x) & 0xfc00) == 0xdc00)) {
                std::string message = std::string("Unexpected unicode low surrogate found in string: ") + c;
                throw std::runtime_error(message);
              }

              // encode as UTF-8
              if (x < 0x800) {
                string.push_back(0xc0 | (x >> 6));
                string.push_back(0x80 | (x & 0x3f));
              } else {
                string.push_back(0xe0 | (x >> 12));
                string.push_back(0x80 | ((x >> 6) & 0x3f));
                string.push_back(0x80 | (x & 0x3f));
              }
              break;
            }
            default: {
              std::string message = std::string("Unrecognized escape sequence found in string: \\") + c;
              throw std::runtime_error(message);
            }
         }
      } else {
        if (unicode_multi != 0) {
        std::string message = std::string("Missing unicode low surrogate in string: ") + c;
        throw std::runtime_error(message);
        }
        string.push_back(c);
      }

      i++;
    }

    // all's well if we made it here
    return string;
  }


static std::string matchString2(const std::string& raw) {
  std::stringstream stream;
  stream << '"';

  const std::string& s = raw;
  std::string::const_iterator it(s.begin()), itEnd(s.end());
  for (; it != itEnd; ++it) {
    // check for UTF-8 unicode encoding
    unsigned char u = static_cast<unsigned char>(*it);
    if (u & 0xc0) {
      if ((u & 0xe0) == 0xc0) {
        // two-character sequence
        int x = (*it & 0x1f) << 6;
        if ((it + 1) == itEnd) {
            stream << *it; continue;
        }
        u = static_cast<unsigned char>(*(it + 1));
        if ((u & 0xc0) == 0x80) {
          x |= u & 0x3f;
          stream << "\\u" << std::hex << std::setfill('0') << std::setw(4) << x;
          ++it;
          continue;
        }

      } else if ((u & 0xf0) == 0xe0) {
        // three-character sequence
        int x = (u & 0x0f) << 12;
        if ((it + 1) == itEnd) {
          stream << *it; continue;
        }
        u = static_cast<unsigned char>(*(it + 1));
        if ((u & 0xc0) == 0x80) {
          x |= (u & 0x3f) << 6;
          if ((it + 2) == itEnd) {
            stream << *it; continue;
          }
          u = static_cast<unsigned char>(*(it + 2));
          if ((u & 0xc0) == 0x80) {
            x |= u & 0x3f;
            stream << "\\u" << std::hex << std::setfill('0')
                << std::setw(4) << x;
            it = it + 2;
            continue;
          }
        }
      }
    }

    switch (*it) {
      case '"':         stream << "\\\"";   break;
      case '\\':        stream << "\\\\";   break;
      case '\b':        stream << "\\b";    break;
      case '\f':        stream << "\\f";    break;
      case '\n':        stream << "\\n";    break;
      case '\r':        stream << "\\r";    break;
      case '\t':        stream << "\\t";    break;
      default:          stream << *it;      break;
    }
  }

  stream << '"';
  return stream.str();
}

};
}  // namespace mros

#endif

