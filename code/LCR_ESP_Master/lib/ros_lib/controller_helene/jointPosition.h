#ifndef _ROS_controller_helene_jointPosition_h
#define _ROS_controller_helene_jointPosition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace controller_helene
{

  class jointPosition : public ros::Msg
  {
    public:
      typedef int32_t _joint1_type;
      _joint1_type joint1;
      typedef int32_t _joint2_type;
      _joint2_type joint2;
      typedef int32_t _joint3_type;
      _joint3_type joint3;
      typedef int32_t _joint4_type;
      _joint4_type joint4;
      typedef int32_t _joint5_type;
      _joint5_type joint5;
      typedef int32_t _joint6_type;
      _joint6_type joint6;

    jointPosition():
      joint1(0),
      joint2(0),
      joint3(0),
      joint4(0),
      joint5(0),
      joint6(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_joint1;
      u_joint1.real = this->joint1;
      *(outbuffer + offset + 0) = (u_joint1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint1);
      union {
        int32_t real;
        uint32_t base;
      } u_joint2;
      u_joint2.real = this->joint2;
      *(outbuffer + offset + 0) = (u_joint2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint2);
      union {
        int32_t real;
        uint32_t base;
      } u_joint3;
      u_joint3.real = this->joint3;
      *(outbuffer + offset + 0) = (u_joint3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint3);
      union {
        int32_t real;
        uint32_t base;
      } u_joint4;
      u_joint4.real = this->joint4;
      *(outbuffer + offset + 0) = (u_joint4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint4);
      union {
        int32_t real;
        uint32_t base;
      } u_joint5;
      u_joint5.real = this->joint5;
      *(outbuffer + offset + 0) = (u_joint5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint5);
      union {
        int32_t real;
        uint32_t base;
      } u_joint6;
      u_joint6.real = this->joint6;
      *(outbuffer + offset + 0) = (u_joint6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint6.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint6);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_joint1;
      u_joint1.base = 0;
      u_joint1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint1 = u_joint1.real;
      offset += sizeof(this->joint1);
      union {
        int32_t real;
        uint32_t base;
      } u_joint2;
      u_joint2.base = 0;
      u_joint2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint2 = u_joint2.real;
      offset += sizeof(this->joint2);
      union {
        int32_t real;
        uint32_t base;
      } u_joint3;
      u_joint3.base = 0;
      u_joint3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint3 = u_joint3.real;
      offset += sizeof(this->joint3);
      union {
        int32_t real;
        uint32_t base;
      } u_joint4;
      u_joint4.base = 0;
      u_joint4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint4 = u_joint4.real;
      offset += sizeof(this->joint4);
      union {
        int32_t real;
        uint32_t base;
      } u_joint5;
      u_joint5.base = 0;
      u_joint5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint5 = u_joint5.real;
      offset += sizeof(this->joint5);
      union {
        int32_t real;
        uint32_t base;
      } u_joint6;
      u_joint6.base = 0;
      u_joint6.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint6.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint6.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint6.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint6 = u_joint6.real;
      offset += sizeof(this->joint6);
     return offset;
    }

    virtual const char * getType() override { return "controller_helene/jointPosition"; };
    virtual const char * getMD5() override { return "956f1cde90e807d13d6f744edcd8a099"; };

  };

}
#endif
