#ifndef _ROS_droneMsgsROS_UltrasonicRangeArduino_h
#define _ROS_droneMsgsROS_UltrasonicRangeArduino_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/UInt8.h"

namespace droneMsgsROS
{

  class UltrasonicRangeArduino : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t id;
      std_msgs::UInt8 range;
      std_msgs::UInt8 freq;
      bool ok;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      offset += this->range.serialize(outbuffer + offset);
      offset += this->freq.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.real = this->ok;
      *(outbuffer + offset + 0) = (u_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      offset += this->range.deserialize(inbuffer + offset);
      offset += this->freq.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.base = 0;
      u_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ok = u_ok.real;
      offset += sizeof(this->ok);
     return offset;
    }

    const char * getType(){ return "droneMsgsROS/UltrasonicRangeArduino"; };
    const char * getMD5(){ return "4ef4f38100122c42e7816667ccbcc205"; };

  };

}
#endif