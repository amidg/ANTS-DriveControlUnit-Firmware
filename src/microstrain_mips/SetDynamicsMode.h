#ifndef _ROS_SERVICE_SetDynamicsMode_h
#define _ROS_SERVICE_SetDynamicsMode_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace microstrain_mips
{

static const char SETDYNAMICSMODE[] = "microstrain_mips/SetDynamicsMode";

  class SetDynamicsModeRequest : public ros::Msg
  {
    public:
      typedef int8_t _mode_type;
      _mode_type mode;
      enum { PORTABLE = 1 };
      enum { AUTOMOTIVE = 2 };
      enum { AIRBORNE = 3 };

    SetDynamicsModeRequest():
      mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
     return offset;
    }

    const char * getType(){ return SETDYNAMICSMODE; };
    const char * getMD5(){ return "a674809088a82424826d8e499e5a94c7"; };

  };

  class SetDynamicsModeResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetDynamicsModeResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SETDYNAMICSMODE; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetDynamicsMode {
    public:
    typedef SetDynamicsModeRequest Request;
    typedef SetDynamicsModeResponse Response;
  };

}
#endif
