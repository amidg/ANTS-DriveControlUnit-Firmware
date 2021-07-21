#ifndef _ROS_SERVICE_SetMagDipAdaptiveVals_h
#define _ROS_SERVICE_SetMagDipAdaptiveVals_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace microstrain_mips
{

static const char SETMAGDIPADAPTIVEVALS[] = "microstrain_mips/SetMagDipAdaptiveVals";

  class SetMagDipAdaptiveValsRequest : public ros::Msg
  {
    public:
      typedef float _enable_type;
      _enable_type enable;
      typedef float _low_pass_cutoff_type;
      _low_pass_cutoff_type low_pass_cutoff;
      typedef float _min_1sigma_type;
      _min_1sigma_type min_1sigma;
      typedef float _high_limit_type;
      _high_limit_type high_limit;
      typedef float _high_limit_1sigma_type;
      _high_limit_1sigma_type high_limit_1sigma;

    SetMagDipAdaptiveValsRequest():
      enable(0),
      low_pass_cutoff(0),
      min_1sigma(0),
      high_limit(0),
      high_limit_1sigma(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_enable.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_enable.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_enable.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->enable);
      union {
        float real;
        uint32_t base;
      } u_low_pass_cutoff;
      u_low_pass_cutoff.real = this->low_pass_cutoff;
      *(outbuffer + offset + 0) = (u_low_pass_cutoff.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_low_pass_cutoff.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_low_pass_cutoff.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_low_pass_cutoff.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->low_pass_cutoff);
      union {
        float real;
        uint32_t base;
      } u_min_1sigma;
      u_min_1sigma.real = this->min_1sigma;
      *(outbuffer + offset + 0) = (u_min_1sigma.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_1sigma.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_1sigma.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_1sigma.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_1sigma);
      union {
        float real;
        uint32_t base;
      } u_high_limit;
      u_high_limit.real = this->high_limit;
      *(outbuffer + offset + 0) = (u_high_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_high_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_high_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_high_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->high_limit);
      union {
        float real;
        uint32_t base;
      } u_high_limit_1sigma;
      u_high_limit_1sigma.real = this->high_limit_1sigma;
      *(outbuffer + offset + 0) = (u_high_limit_1sigma.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_high_limit_1sigma.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_high_limit_1sigma.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_high_limit_1sigma.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->high_limit_1sigma);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_enable.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_enable.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_enable.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
      union {
        float real;
        uint32_t base;
      } u_low_pass_cutoff;
      u_low_pass_cutoff.base = 0;
      u_low_pass_cutoff.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_low_pass_cutoff.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_low_pass_cutoff.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_low_pass_cutoff.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->low_pass_cutoff = u_low_pass_cutoff.real;
      offset += sizeof(this->low_pass_cutoff);
      union {
        float real;
        uint32_t base;
      } u_min_1sigma;
      u_min_1sigma.base = 0;
      u_min_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_1sigma = u_min_1sigma.real;
      offset += sizeof(this->min_1sigma);
      union {
        float real;
        uint32_t base;
      } u_high_limit;
      u_high_limit.base = 0;
      u_high_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_high_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_high_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_high_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->high_limit = u_high_limit.real;
      offset += sizeof(this->high_limit);
      union {
        float real;
        uint32_t base;
      } u_high_limit_1sigma;
      u_high_limit_1sigma.base = 0;
      u_high_limit_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_high_limit_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_high_limit_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_high_limit_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->high_limit_1sigma = u_high_limit_1sigma.real;
      offset += sizeof(this->high_limit_1sigma);
     return offset;
    }

    const char * getType(){ return SETMAGDIPADAPTIVEVALS; };
    const char * getMD5(){ return "04ce6c1f346dae412b474a781c1703c9"; };

  };

  class SetMagDipAdaptiveValsResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetMagDipAdaptiveValsResponse():
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

    const char * getType(){ return SETMAGDIPADAPTIVEVALS; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetMagDipAdaptiveVals {
    public:
    typedef SetMagDipAdaptiveValsRequest Request;
    typedef SetMagDipAdaptiveValsResponse Response;
  };

}
#endif
