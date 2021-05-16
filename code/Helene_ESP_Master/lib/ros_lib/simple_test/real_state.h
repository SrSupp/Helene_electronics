#ifndef _ROS_simple_test_real_state_h
#define _ROS_simple_test_real_state_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace simple_test
{

  class real_state : public ros::Msg
  {
    public:
      typedef float _x_actual_type;
      _x_actual_type x_actual;
      typedef float _v_actual_type;
      _v_actual_type v_actual;

    real_state():
      x_actual(0),
      v_actual(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x_actual;
      u_x_actual.real = this->x_actual;
      *(outbuffer + offset + 0) = (u_x_actual.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_actual.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_actual.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_actual.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_actual);
      union {
        float real;
        uint32_t base;
      } u_v_actual;
      u_v_actual.real = this->v_actual;
      *(outbuffer + offset + 0) = (u_v_actual.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_actual.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_actual.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_actual.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_actual);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x_actual;
      u_x_actual.base = 0;
      u_x_actual.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_actual.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_actual.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_actual.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_actual = u_x_actual.real;
      offset += sizeof(this->x_actual);
      union {
        float real;
        uint32_t base;
      } u_v_actual;
      u_v_actual.base = 0;
      u_v_actual.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_actual.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_actual.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_actual.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_actual = u_v_actual.real;
      offset += sizeof(this->v_actual);
     return offset;
    }

    const char * getType(){ return "simple_test/real_state"; };
    const char * getMD5(){ return "b2376b6ada9267b77d1135407eb5512f"; };

  };

}
#endif
