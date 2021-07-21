#ifndef _ROS_SERVICE_SetConingScullingComp_h
#define _ROS_SERVICE_SetConingScullingComp_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace microstrain_mips
{

static const char SETCONINGSCULLINGCOMP[] = "microstrain_mips/SetConingScullingComp";

  class SetConingScullingCompRequest : public ros::Msg
  {
    public:
      typedef int8_t _enable_type;
      _enable_type enable;

    SetConingScullingCompRequest():
      enable(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
     return offset;
    }

    const char * getType(){ return SETCONINGSCULLINGCOMP; };
    const char * getMD5(){ return "ac6526613895843e4f8f058d9ae9c5ce"; };

  };

  class SetConingScullingCompResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetConingScullingCompResponse():
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

    const char * getType(){ return SETCONINGSCULLINGCOMP; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetConingScullingComp {
    public:
    typedef SetConingScullingCompRequest Request;
    typedef SetConingScullingCompResponse Response;
  };

}
#endif
