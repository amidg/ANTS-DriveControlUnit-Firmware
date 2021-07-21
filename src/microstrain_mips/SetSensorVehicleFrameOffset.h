#ifndef _ROS_SERVICE_SetSensorVehicleFrameOffset_h
#define _ROS_SERVICE_SetSensorVehicleFrameOffset_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace microstrain_mips
{

static const char SETSENSORVEHICLEFRAMEOFFSET[] = "microstrain_mips/SetSensorVehicleFrameOffset";

  class SetSensorVehicleFrameOffsetRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _offset_type;
      _offset_type offset;

    SetSensorVehicleFrameOffsetRequest():
      offset()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->offset.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->offset.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETSENSORVEHICLEFRAMEOFFSET; };
    const char * getMD5(){ return "de19cca9344eb5bfedb7e55986a47f2e"; };

  };

  class SetSensorVehicleFrameOffsetResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetSensorVehicleFrameOffsetResponse():
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

    const char * getType(){ return SETSENSORVEHICLEFRAMEOFFSET; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetSensorVehicleFrameOffset {
    public:
    typedef SetSensorVehicleFrameOffsetRequest Request;
    typedef SetSensorVehicleFrameOffsetResponse Response;
  };

}
#endif
