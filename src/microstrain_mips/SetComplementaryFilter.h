#ifndef _ROS_SERVICE_SetComplementaryFilter_h
#define _ROS_SERVICE_SetComplementaryFilter_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace microstrain_mips
{

static const char SETCOMPLEMENTARYFILTER[] = "microstrain_mips/SetComplementaryFilter";

  class SetComplementaryFilterRequest : public ros::Msg
  {
    public:
      typedef int8_t _north_comp_enable_type;
      _north_comp_enable_type north_comp_enable;
      typedef int8_t _up_comp_enable_type;
      _up_comp_enable_type up_comp_enable;
      typedef float _north_comp_time_const_type;
      _north_comp_time_const_type north_comp_time_const;
      typedef float _up_comp_time_const_type;
      _up_comp_time_const_type up_comp_time_const;

    SetComplementaryFilterRequest():
      north_comp_enable(0),
      up_comp_enable(0),
      north_comp_time_const(0),
      up_comp_time_const(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_north_comp_enable;
      u_north_comp_enable.real = this->north_comp_enable;
      *(outbuffer + offset + 0) = (u_north_comp_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->north_comp_enable);
      union {
        int8_t real;
        uint8_t base;
      } u_up_comp_enable;
      u_up_comp_enable.real = this->up_comp_enable;
      *(outbuffer + offset + 0) = (u_up_comp_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->up_comp_enable);
      union {
        float real;
        uint32_t base;
      } u_north_comp_time_const;
      u_north_comp_time_const.real = this->north_comp_time_const;
      *(outbuffer + offset + 0) = (u_north_comp_time_const.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_north_comp_time_const.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_north_comp_time_const.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_north_comp_time_const.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->north_comp_time_const);
      union {
        float real;
        uint32_t base;
      } u_up_comp_time_const;
      u_up_comp_time_const.real = this->up_comp_time_const;
      *(outbuffer + offset + 0) = (u_up_comp_time_const.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_up_comp_time_const.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_up_comp_time_const.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_up_comp_time_const.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->up_comp_time_const);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_north_comp_enable;
      u_north_comp_enable.base = 0;
      u_north_comp_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->north_comp_enable = u_north_comp_enable.real;
      offset += sizeof(this->north_comp_enable);
      union {
        int8_t real;
        uint8_t base;
      } u_up_comp_enable;
      u_up_comp_enable.base = 0;
      u_up_comp_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->up_comp_enable = u_up_comp_enable.real;
      offset += sizeof(this->up_comp_enable);
      union {
        float real;
        uint32_t base;
      } u_north_comp_time_const;
      u_north_comp_time_const.base = 0;
      u_north_comp_time_const.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_north_comp_time_const.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_north_comp_time_const.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_north_comp_time_const.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->north_comp_time_const = u_north_comp_time_const.real;
      offset += sizeof(this->north_comp_time_const);
      union {
        float real;
        uint32_t base;
      } u_up_comp_time_const;
      u_up_comp_time_const.base = 0;
      u_up_comp_time_const.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_up_comp_time_const.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_up_comp_time_const.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_up_comp_time_const.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->up_comp_time_const = u_up_comp_time_const.real;
      offset += sizeof(this->up_comp_time_const);
     return offset;
    }

    const char * getType(){ return SETCOMPLEMENTARYFILTER; };
    const char * getMD5(){ return "45ff6c0b3e0fce49632971cc1d368393"; };

  };

  class SetComplementaryFilterResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetComplementaryFilterResponse():
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

    const char * getType(){ return SETCOMPLEMENTARYFILTER; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetComplementaryFilter {
    public:
    typedef SetComplementaryFilterRequest Request;
    typedef SetComplementaryFilterResponse Response;
  };

}
#endif
