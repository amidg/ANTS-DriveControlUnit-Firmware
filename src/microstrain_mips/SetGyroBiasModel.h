#ifndef _ROS_SERVICE_SetGyroBiasModel_h
#define _ROS_SERVICE_SetGyroBiasModel_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace microstrain_mips
{

static const char SETGYROBIASMODEL[] = "microstrain_mips/SetGyroBiasModel";

  class SetGyroBiasModelRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _noise_vector_type;
      _noise_vector_type noise_vector;
      typedef geometry_msgs::Vector3 _beta_vector_type;
      _beta_vector_type beta_vector;

    SetGyroBiasModelRequest():
      noise_vector(),
      beta_vector()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->noise_vector.serialize(outbuffer + offset);
      offset += this->beta_vector.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->noise_vector.deserialize(inbuffer + offset);
      offset += this->beta_vector.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETGYROBIASMODEL; };
    const char * getMD5(){ return "89f0f0cd6d52eb4058eea24a6259c0d1"; };

  };

  class SetGyroBiasModelResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetGyroBiasModelResponse():
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

    const char * getType(){ return SETGYROBIASMODEL; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetGyroBiasModel {
    public:
    typedef SetGyroBiasModelRequest Request;
    typedef SetGyroBiasModelResponse Response;
  };

}
#endif
