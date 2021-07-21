#ifndef _ROS_SERVICE_SetFilterEuler_h
#define _ROS_SERVICE_SetFilterEuler_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace microstrain_mips
{

static const char SETFILTEREULER[] = "microstrain_mips/SetFilterEuler";

  class SetFilterEulerRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _angle_type;
      _angle_type angle;

    SetFilterEulerRequest():
      angle()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->angle.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->angle.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETFILTEREULER; };
    const char * getMD5(){ return "d1832802105b934d111265eaec60b073"; };

  };

  class SetFilterEulerResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetFilterEulerResponse():
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

    const char * getType(){ return SETFILTEREULER; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetFilterEuler {
    public:
    typedef SetFilterEulerRequest Request;
    typedef SetFilterEulerResponse Response;
  };

}
#endif
