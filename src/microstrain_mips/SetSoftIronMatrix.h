#ifndef _ROS_SERVICE_SetSoftIronMatrix_h
#define _ROS_SERVICE_SetSoftIronMatrix_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace microstrain_mips
{

static const char SETSOFTIRONMATRIX[] = "microstrain_mips/SetSoftIronMatrix";

  class SetSoftIronMatrixRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _soft_iron_1_type;
      _soft_iron_1_type soft_iron_1;
      typedef geometry_msgs::Vector3 _soft_iron_2_type;
      _soft_iron_2_type soft_iron_2;
      typedef geometry_msgs::Vector3 _soft_iron_3_type;
      _soft_iron_3_type soft_iron_3;

    SetSoftIronMatrixRequest():
      soft_iron_1(),
      soft_iron_2(),
      soft_iron_3()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->soft_iron_1.serialize(outbuffer + offset);
      offset += this->soft_iron_2.serialize(outbuffer + offset);
      offset += this->soft_iron_3.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->soft_iron_1.deserialize(inbuffer + offset);
      offset += this->soft_iron_2.deserialize(inbuffer + offset);
      offset += this->soft_iron_3.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETSOFTIRONMATRIX; };
    const char * getMD5(){ return "7f7e0181b730797800abbfc3104a7df3"; };

  };

  class SetSoftIronMatrixResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetSoftIronMatrixResponse():
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

    const char * getType(){ return SETSOFTIRONMATRIX; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetSoftIronMatrix {
    public:
    typedef SetSoftIronMatrixRequest Request;
    typedef SetSoftIronMatrixResponse Response;
  };

}
#endif
