#ifndef _ROS_driving_signals_Signals_h
#define _ROS_driving_signals_Signals_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace driving_signals
{

  class Signals : public ros::Msg
  {
    public:
      typedef uint8_t _turn_right_type;
      _turn_right_type turn_right;
      typedef uint8_t _turn_left_type;
      _turn_left_type turn_left;
      typedef uint8_t _backward_type;
      _backward_type backward;
      typedef uint8_t _forward_type;
      _forward_type forward;
      typedef uint8_t _e_stop_type;
      _e_stop_type e_stop;

    Signals():
      turn_right(0),
      turn_left(0),
      backward(0),
      forward(0),
      e_stop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->turn_right >> (8 * 0)) & 0xFF;
      offset += sizeof(this->turn_right);
      *(outbuffer + offset + 0) = (this->turn_left >> (8 * 0)) & 0xFF;
      offset += sizeof(this->turn_left);
      *(outbuffer + offset + 0) = (this->backward >> (8 * 0)) & 0xFF;
      offset += sizeof(this->backward);
      *(outbuffer + offset + 0) = (this->forward >> (8 * 0)) & 0xFF;
      offset += sizeof(this->forward);
      *(outbuffer + offset + 0) = (this->e_stop >> (8 * 0)) & 0xFF;
      offset += sizeof(this->e_stop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->turn_right =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->turn_right);
      this->turn_left =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->turn_left);
      this->backward =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->backward);
      this->forward =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->forward);
      this->e_stop =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->e_stop);
     return offset;
    }

    const char * getType(){ return "driving_signals/Signals"; };
    const char * getMD5(){ return "8bf27ecde9aacf732475165d1f570c5f"; };

  };

}
#endif
