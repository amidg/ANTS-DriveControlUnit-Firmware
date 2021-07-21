#ifndef _ROS_SERVICE_SetHardIronValues_h
#define _ROS_SERVICE_SetHardIronValues_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace microstrain_mips
{

static const char SETHARDIRONVALUES[] = "microstrain_mips/SetHardIronValues";

  class SetHardIronValuesRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _bias_type;
      _bias_type bias;

    SetHardIronValuesRequest():
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

    const char * getType(){ return SETHARDIRONVALUES; };
    const char * getMD5(){ return "af1f260075d9ba9bd73ca10c6a45df07"; };

  };

  class SetHardIronValuesResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetHardIronValuesResponse():
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

    const char * getType(){ return SETHARDIRONVALUES; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetHardIronValues {
    public:
    typedef SetHardIronValuesRequest Request;
    typedef SetHardIronValuesResponse Response;
  };

}
#endif
