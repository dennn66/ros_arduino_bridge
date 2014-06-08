#ifndef _ROS_ros_arduino_msgs_AnalogFloat_h
#define _ROS_ros_arduino_msgs_AnalogFloat_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ros_arduino_msgs
{

  class AnalogFloat : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float value;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      int32_t * val_value = (int32_t *) &(this->value);
      int32_t exp_value = (((*val_value)>>23)&255);
      if(exp_value != 0)
        exp_value += 1023-127;
      int32_t sig_value = *val_value;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_value<<5) & 0xff;
      *(outbuffer + offset++) = (sig_value>>3) & 0xff;
      *(outbuffer + offset++) = (sig_value>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_value<<4) & 0xF0) | ((sig_value>>19)&0x0F);
      *(outbuffer + offset++) = (exp_value>>4) & 0x7F;
      if(this->value < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t * val_value = (uint32_t*) &(this->value);
      offset += 3;
      *val_value = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_value |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_value |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_value |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_value = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_value |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_value !=0)
        *val_value |= ((exp_value)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->value = -this->value;
     return offset;
    }

    const char * getType(){ return "ros_arduino_msgs/AnalogFloat"; };
    const char * getMD5(){ return "d053817de0764f9ee90dbc89c4cdd751"; };

  };

}
#endif