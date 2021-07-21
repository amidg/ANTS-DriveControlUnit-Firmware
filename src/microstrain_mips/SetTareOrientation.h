#ifndef _ROS_SERVICE_SetTareOrientation_h
#define _ROS_SERVICE_SetTareOrientation_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace microstrain_mips
{

static const char SETTAREORIENTATION[] = "microstrain_mips/SetTareOrientation";

  class SetTareOrientationRequest : public ros::Msg
  {
    public:
      typedef int8_t _axis_type;
      _axis_type axis;

    SetTareOrientationRequest():
      axis(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_axis;
      u_axis.real = this->axis;
      *(outbuffer + offset + 0) = (u_axis.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->axis);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_axis;
      u_axis.base = 0;
      u_axis.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->axis = u_axis.real;
      offset += sizeof(this->axis);
     return offset;
    }

    const char * getType(){ return SETTAREORIENTATION; };
    const char * getMD5(){ return "ea297538e481c0b3228cef125a01ff35"; };

  };

  class SetTareOrientationResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetTareOrientationResponse():
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

    const char * getType(){ return SETTAREORIENTATION; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetTareOrientation {
    public:
    typedef SetTareOrientationRequest Request;
    typedef SetTareOrientationResponse Response;
  };

}
#endif
