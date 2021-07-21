#ifndef _ROS_microstrain_mips_status_msg_h
#define _ROS_microstrain_mips_status_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace microstrain_mips
{

  class status_msg : public ros::Msg
  {
    public:
      typedef uint16_t _device_model_type;
      _device_model_type device_model;
      typedef uint8_t _status_selector_type;
      _status_selector_type status_selector;
      typedef uint32_t _status_flags_type;
      _status_flags_type status_flags;
      typedef uint16_t _system_state_type;
      _system_state_type system_state;
      typedef uint32_t _system_timer_ms_type;
      _system_timer_ms_type system_timer_ms;
      typedef uint8_t _gps_power_on_type;
      _gps_power_on_type gps_power_on;
      typedef uint32_t _num_gps_pps_triggers_type;
      _num_gps_pps_triggers_type num_gps_pps_triggers;
      typedef uint32_t _last_gps_pps_trigger_ms_type;
      _last_gps_pps_trigger_ms_type last_gps_pps_trigger_ms;
      typedef uint8_t _imu_stream_enabled_type;
      _imu_stream_enabled_type imu_stream_enabled;
      typedef uint8_t _gps_stream_enabled_type;
      _gps_stream_enabled_type gps_stream_enabled;
      typedef uint8_t _filter_stream_enabled_type;
      _filter_stream_enabled_type filter_stream_enabled;
      typedef uint32_t _imu_dropped_packets_type;
      _imu_dropped_packets_type imu_dropped_packets;
      typedef uint32_t _gps_dropped_packets_type;
      _gps_dropped_packets_type gps_dropped_packets;
      typedef uint32_t _filter_dropped_packets_type;
      _filter_dropped_packets_type filter_dropped_packets;
      typedef uint32_t _com1_port_bytes_written_type;
      _com1_port_bytes_written_type com1_port_bytes_written;
      typedef uint32_t _com1_port_bytes_read_type;
      _com1_port_bytes_read_type com1_port_bytes_read;
      typedef uint32_t _com1_port_write_overruns_type;
      _com1_port_write_overruns_type com1_port_write_overruns;
      typedef uint32_t _com1_port_read_overruns_type;
      _com1_port_read_overruns_type com1_port_read_overruns;
      typedef uint32_t _imu_parser_errors_type;
      _imu_parser_errors_type imu_parser_errors;
      typedef uint32_t _imu_message_count_type;
      _imu_message_count_type imu_message_count;
      typedef uint32_t _imu_last_message_ms_type;
      _imu_last_message_ms_type imu_last_message_ms;
      typedef uint32_t _gps_parser_errors_type;
      _gps_parser_errors_type gps_parser_errors;
      typedef uint32_t _gps_message_count_type;
      _gps_message_count_type gps_message_count;
      typedef uint32_t _gps_last_message_ms_type;
      _gps_last_message_ms_type gps_last_message_ms;

    status_msg():
      device_model(0),
      status_selector(0),
      status_flags(0),
      system_state(0),
      system_timer_ms(0),
      gps_power_on(0),
      num_gps_pps_triggers(0),
      last_gps_pps_trigger_ms(0),
      imu_stream_enabled(0),
      gps_stream_enabled(0),
      filter_stream_enabled(0),
      imu_dropped_packets(0),
      gps_dropped_packets(0),
      filter_dropped_packets(0),
      com1_port_bytes_written(0),
      com1_port_bytes_read(0),
      com1_port_write_overruns(0),
      com1_port_read_overruns(0),
      imu_parser_errors(0),
      imu_message_count(0),
      imu_last_message_ms(0),
      gps_parser_errors(0),
      gps_message_count(0),
      gps_last_message_ms(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->device_model >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->device_model >> (8 * 1)) & 0xFF;
      offset += sizeof(this->device_model);
      *(outbuffer + offset + 0) = (this->status_selector >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status_selector);
      *(outbuffer + offset + 0) = (this->status_flags >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status_flags >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->status_flags >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->status_flags >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status_flags);
      *(outbuffer + offset + 0) = (this->system_state >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->system_state >> (8 * 1)) & 0xFF;
      offset += sizeof(this->system_state);
      *(outbuffer + offset + 0) = (this->system_timer_ms >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->system_timer_ms >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->system_timer_ms >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->system_timer_ms >> (8 * 3)) & 0xFF;
      offset += sizeof(this->system_timer_ms);
      *(outbuffer + offset + 0) = (this->gps_power_on >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps_power_on);
      *(outbuffer + offset + 0) = (this->num_gps_pps_triggers >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num_gps_pps_triggers >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->num_gps_pps_triggers >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->num_gps_pps_triggers >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_gps_pps_triggers);
      *(outbuffer + offset + 0) = (this->last_gps_pps_trigger_ms >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->last_gps_pps_trigger_ms >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->last_gps_pps_trigger_ms >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->last_gps_pps_trigger_ms >> (8 * 3)) & 0xFF;
      offset += sizeof(this->last_gps_pps_trigger_ms);
      *(outbuffer + offset + 0) = (this->imu_stream_enabled >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_stream_enabled);
      *(outbuffer + offset + 0) = (this->gps_stream_enabled >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps_stream_enabled);
      *(outbuffer + offset + 0) = (this->filter_stream_enabled >> (8 * 0)) & 0xFF;
      offset += sizeof(this->filter_stream_enabled);
      *(outbuffer + offset + 0) = (this->imu_dropped_packets >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->imu_dropped_packets >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->imu_dropped_packets >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->imu_dropped_packets >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imu_dropped_packets);
      *(outbuffer + offset + 0) = (this->gps_dropped_packets >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gps_dropped_packets >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gps_dropped_packets >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gps_dropped_packets >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_dropped_packets);
      *(outbuffer + offset + 0) = (this->filter_dropped_packets >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->filter_dropped_packets >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->filter_dropped_packets >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->filter_dropped_packets >> (8 * 3)) & 0xFF;
      offset += sizeof(this->filter_dropped_packets);
      *(outbuffer + offset + 0) = (this->com1_port_bytes_written >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->com1_port_bytes_written >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->com1_port_bytes_written >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->com1_port_bytes_written >> (8 * 3)) & 0xFF;
      offset += sizeof(this->com1_port_bytes_written);
      *(outbuffer + offset + 0) = (this->com1_port_bytes_read >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->com1_port_bytes_read >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->com1_port_bytes_read >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->com1_port_bytes_read >> (8 * 3)) & 0xFF;
      offset += sizeof(this->com1_port_bytes_read);
      *(outbuffer + offset + 0) = (this->com1_port_write_overruns >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->com1_port_write_overruns >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->com1_port_write_overruns >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->com1_port_write_overruns >> (8 * 3)) & 0xFF;
      offset += sizeof(this->com1_port_write_overruns);
      *(outbuffer + offset + 0) = (this->com1_port_read_overruns >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->com1_port_read_overruns >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->com1_port_read_overruns >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->com1_port_read_overruns >> (8 * 3)) & 0xFF;
      offset += sizeof(this->com1_port_read_overruns);
      *(outbuffer + offset + 0) = (this->imu_parser_errors >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->imu_parser_errors >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->imu_parser_errors >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->imu_parser_errors >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imu_parser_errors);
      *(outbuffer + offset + 0) = (this->imu_message_count >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->imu_message_count >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->imu_message_count >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->imu_message_count >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imu_message_count);
      *(outbuffer + offset + 0) = (this->imu_last_message_ms >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->imu_last_message_ms >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->imu_last_message_ms >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->imu_last_message_ms >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imu_last_message_ms);
      *(outbuffer + offset + 0) = (this->gps_parser_errors >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gps_parser_errors >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gps_parser_errors >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gps_parser_errors >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_parser_errors);
      *(outbuffer + offset + 0) = (this->gps_message_count >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gps_message_count >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gps_message_count >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gps_message_count >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_message_count);
      *(outbuffer + offset + 0) = (this->gps_last_message_ms >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gps_last_message_ms >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gps_last_message_ms >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gps_last_message_ms >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_last_message_ms);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->device_model =  ((uint16_t) (*(inbuffer + offset)));
      this->device_model |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->device_model);
      this->status_selector =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status_selector);
      this->status_flags =  ((uint32_t) (*(inbuffer + offset)));
      this->status_flags |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->status_flags |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->status_flags |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->status_flags);
      this->system_state =  ((uint16_t) (*(inbuffer + offset)));
      this->system_state |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->system_state);
      this->system_timer_ms =  ((uint32_t) (*(inbuffer + offset)));
      this->system_timer_ms |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->system_timer_ms |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->system_timer_ms |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->system_timer_ms);
      this->gps_power_on =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gps_power_on);
      this->num_gps_pps_triggers =  ((uint32_t) (*(inbuffer + offset)));
      this->num_gps_pps_triggers |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_gps_pps_triggers |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->num_gps_pps_triggers |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->num_gps_pps_triggers);
      this->last_gps_pps_trigger_ms =  ((uint32_t) (*(inbuffer + offset)));
      this->last_gps_pps_trigger_ms |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->last_gps_pps_trigger_ms |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->last_gps_pps_trigger_ms |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->last_gps_pps_trigger_ms);
      this->imu_stream_enabled =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->imu_stream_enabled);
      this->gps_stream_enabled =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gps_stream_enabled);
      this->filter_stream_enabled =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->filter_stream_enabled);
      this->imu_dropped_packets =  ((uint32_t) (*(inbuffer + offset)));
      this->imu_dropped_packets |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->imu_dropped_packets |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->imu_dropped_packets |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->imu_dropped_packets);
      this->gps_dropped_packets =  ((uint32_t) (*(inbuffer + offset)));
      this->gps_dropped_packets |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gps_dropped_packets |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gps_dropped_packets |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gps_dropped_packets);
      this->filter_dropped_packets =  ((uint32_t) (*(inbuffer + offset)));
      this->filter_dropped_packets |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->filter_dropped_packets |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->filter_dropped_packets |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->filter_dropped_packets);
      this->com1_port_bytes_written =  ((uint32_t) (*(inbuffer + offset)));
      this->com1_port_bytes_written |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->com1_port_bytes_written |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->com1_port_bytes_written |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->com1_port_bytes_written);
      this->com1_port_bytes_read =  ((uint32_t) (*(inbuffer + offset)));
      this->com1_port_bytes_read |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->com1_port_bytes_read |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->com1_port_bytes_read |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->com1_port_bytes_read);
      this->com1_port_write_overruns =  ((uint32_t) (*(inbuffer + offset)));
      this->com1_port_write_overruns |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->com1_port_write_overruns |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->com1_port_write_overruns |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->com1_port_write_overruns);
      this->com1_port_read_overruns =  ((uint32_t) (*(inbuffer + offset)));
      this->com1_port_read_overruns |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->com1_port_read_overruns |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->com1_port_read_overruns |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->com1_port_read_overruns);
      this->imu_parser_errors =  ((uint32_t) (*(inbuffer + offset)));
      this->imu_parser_errors |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->imu_parser_errors |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->imu_parser_errors |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->imu_parser_errors);
      this->imu_message_count =  ((uint32_t) (*(inbuffer + offset)));
      this->imu_message_count |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->imu_message_count |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->imu_message_count |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->imu_message_count);
      this->imu_last_message_ms =  ((uint32_t) (*(inbuffer + offset)));
      this->imu_last_message_ms |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->imu_last_message_ms |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->imu_last_message_ms |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->imu_last_message_ms);
      this->gps_parser_errors =  ((uint32_t) (*(inbuffer + offset)));
      this->gps_parser_errors |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gps_parser_errors |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gps_parser_errors |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gps_parser_errors);
      this->gps_message_count =  ((uint32_t) (*(inbuffer + offset)));
      this->gps_message_count |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gps_message_count |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gps_message_count |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gps_message_count);
      this->gps_last_message_ms =  ((uint32_t) (*(inbuffer + offset)));
      this->gps_last_message_ms |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gps_last_message_ms |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gps_last_message_ms |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gps_last_message_ms);
     return offset;
    }

    const char * getType(){ return "microstrain_mips/status_msg"; };
    const char * getMD5(){ return "b19b8bec6f96c609664c27dbc51ddf60"; };

  };

}
#endif
