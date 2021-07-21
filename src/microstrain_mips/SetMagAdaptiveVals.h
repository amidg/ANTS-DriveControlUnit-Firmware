#ifndef _ROS_SERVICE_SetMagAdaptiveVals_h
#define _ROS_SERVICE_SetMagAdaptiveVals_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace microstrain_mips
{

static const char SETMAGADAPTIVEVALS[] = "microstrain_mips/SetMagAdaptiveVals";

  class SetMagAdaptiveValsRequest : public ros::Msg
  {
    public:
      typedef float _enable_type;
      _enable_type enable;
      typedef float _low_pass_cutoff_type;
      _low_pass_cutoff_type low_pass_cutoff;
      typedef float _min_1sigma_type;
      _min_1sigma_type min_1sigma;
      typedef float _low_limit_type;
      _low_limit_type low_limit;
      typedef float _high_limit_type;
      _high_limit_type high_limit;
      typedef float _low_limit_1sigma_type;
      _low_limit_1sigma_type low_limit_1sigma;
      typedef float _high_limit_1sigma_type;
      _high_limit_1sigma_type high_limit_1sigma;

    SetMagAdaptiveValsRequest():
      enable(0),
      low_pass_cutoff(0),
      min_1sigma(0),
      low_limit(0),
      high_limit(0),
      low_limit_1sigma(0),
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
      } u_low_limit;
      u_low_limit.real = this->low_limit;
      *(outbuffer + offset + 0) = (u_low_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_low_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_low_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_low_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->low_limit);
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
      } u_low_limit_1sigma;
      u_low_limit_1sigma.real = this->low_limit_1sigma;
      *(outbuffer + offset + 0) = (u_low_limit_1sigma.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_low_limit_1sigma.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_low_limit_1sigma.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_low_limit_1sigma.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->low_limit_1sigma);
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
      } u_low_limit;
      u_low_limit.base = 0;
      u_low_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_low_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_low_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_low_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->low_limit = u_low_limit.real;
      offset += sizeof(this->low_limit);
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
      } u_low_limit_1sigma;
      u_low_limit_1sigma.base = 0;
      u_low_limit_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_low_limit_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_low_limit_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_low_limit_1sigma.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->low_limit_1sigma = u_low_limit_1sigma.real;
      offset += sizeof(this->low_limit_1sigma);
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

    const char * getType(){ return SETMAGADAPTIVEVALS; };
    const char * getMD5(){ return "ea0a038591e6f2c00f93fee5561157dc"; };

  };

  class SetMagAdaptiveValsResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetMagAdaptiveValsResponse():
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

    const char * getType(){ return SETMAGADAPTIVEVALS; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetMagAdaptiveVals {
    public:
    typedef SetMagAdaptiveValsRequest Request;
    typedef SetMagAdaptiveValsResponse Response;
  };

}
#endif
