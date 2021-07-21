#ifndef _ROS_SERVICE_SetGyroBias_h
#define _ROS_SERVICE_SetGyroBias_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace microstrain_mips
{

static const char SETGYROBIAS[] = "microstrain_mips/SetGyroBias";

  class SetGyroBiasRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _bias_type;
      _bias_type bias;

    SetGyroBiasRequest():
      bias()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->bias.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->bias.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETGYROBIAS; };
    const char * getMD5(){ return "af1f260075d9ba9bd73ca10c6a45df07"; };

  };

  class SetGyroBiasResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetGyroBiasResponse():
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

    const char * getType(){ return SETGYROBIAS; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetGyroBias {
    public:
    typedef SetGyroBiasRequest Request;
    typedef SetGyroBiasResponse Response;
  };

}
#endif
