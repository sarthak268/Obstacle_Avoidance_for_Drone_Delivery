#ifndef _ROS_SERVICE_ultrasonicArduinoService_h
#define _ROS_SERVICE_ultrasonicArduinoService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "droneMsgsROS/UltrasonicRangeArduino.h"
#include "std_msgs/Int8.h"

namespace droneMsgsROS
{

static const char ULTRASONICARDUINOSERVICE[] = "droneMsgsROS/ultrasonicArduinoService";

  class ultrasonicArduinoServiceRequest : public ros::Msg
  {
    public:
      std_msgs::Int8 req_frequency;
      std_msgs::Int8 req_mode;
      std_msgs::Int8 req_update;
      uint8_t req_id;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->req_frequency.serialize(outbuffer + offset);
      offset += this->req_mode.serialize(outbuffer + offset);
      offset += this->req_update.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->req_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->req_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->req_frequency.deserialize(inbuffer + offset);
      offset += this->req_mode.deserialize(inbuffer + offset);
      offset += this->req_update.deserialize(inbuffer + offset);
      this->req_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->req_id);
     return offset;
    }

    const char * getType(){ return ULTRASONICARDUINOSERVICE; };
    const char * getMD5(){ return "13874fca88e22ba6b354356389226db4"; };

  };

  class ultrasonicArduinoServiceResponse : public ros::Msg
  {
    public:
      droneMsgsROS::UltrasonicRangeArduino range;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->range.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->range.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return ULTRASONICARDUINOSERVICE; };
    const char * getMD5(){ return "3b3cef73ffd93aa2d48c1875ad1df4f8"; };

  };

  class ultrasonicArduinoService {
    public:
    typedef ultrasonicArduinoServiceRequest Request;
    typedef ultrasonicArduinoServiceResponse Response;
  };

}
#endif
