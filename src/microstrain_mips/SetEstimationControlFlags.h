#ifndef _ROS_SERVICE_SetEstimationControlFlags_h
#define _ROS_SERVICE_SetEstimationControlFlags_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace microstrain_mips
{

static const char SETESTIMATIONCONTROLFLAGS[] = "microstrain_mips/SetEstimationControlFlags";

  class SetEstimationControlFlagsRequest : public ros::Msg
  {
    public:
      typedef int8_t _flag_type;
      _flag_type flag;
      enum { GYRO_BIAS_ESTIMATION = 1 };
      enum { HARD_IRON_AUTO_CALIBRATION = 32 };
      enum { SOFT_IRON_AUTO_CALIBRATION = 64 };

    SetEstimationControlFlagsRequest():
      flag(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_flag;
      u_flag.real = this->flag;
      *(outbuffer + offset + 0) = (u_flag.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flag);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_flag;
      u_flag.base = 0;
      u_flag.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->flag = u_flag.real;
      offset += sizeof(this->flag);
     return offset;
    }

    const char * getType(){ return SETESTIMATIONCONTROLFLAGS; };
    const char * getMD5(){ return "ad1023f6ef4358dd8f820bdc9b6f40cd"; };

  };

  class SetEstimationControlFlagsResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetEstimationControlFlagsResponse():
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

    const char * getType(){ return SETESTIMATIONCONTROLFLAGS; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetEstimationControlFlags {
    public:
    typedef SetEstimationControlFlagsRequest Request;
    typedef SetEstimationControlFlagsResponse Response;
  };

}
#endif
