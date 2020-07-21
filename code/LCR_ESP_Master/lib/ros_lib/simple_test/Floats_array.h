#ifndef _ROS_SERVICE_Floats_array_h
#define _ROS_SERVICE_Floats_array_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace simple_test
{

static const char FLOATS_ARRAY[] = "simple_test/Floats_array";

  class Floats_arrayRequest : public ros::Msg
  {
    public:
      typedef float _req_type;
      _req_type req;

    Floats_arrayRequest():
      req(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_req;
      u_req.real = this->req;
      *(outbuffer + offset + 0) = (u_req.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_req.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_req.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_req.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->req);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_req;
      u_req.base = 0;
      u_req.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_req.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_req.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_req.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->req = u_req.real;
      offset += sizeof(this->req);
     return offset;
    }

    const char * getType(){ return FLOATS_ARRAY; };
    const char * getMD5(){ return "d9e2e500167ba94696705f008dcefd57"; };

  };

  class Floats_arrayResponse : public ros::Msg
  {
    public:
      uint32_t res_length;
      typedef float _res_type;
      _res_type st_res;
      _res_type * res;

    Floats_arrayResponse():
      res_length(0), res(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->res_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->res_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->res_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->res_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->res_length);
      for( uint32_t i = 0; i < res_length; i++){
      union {
        float real;
        uint32_t base;
      } u_resi;
      u_resi.real = this->res[i];
      *(outbuffer + offset + 0) = (u_resi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_resi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_resi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_resi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->res[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t res_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      res_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      res_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      res_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->res_length);
      if(res_lengthT > res_length)
        this->res = (float*)realloc(this->res, res_lengthT * sizeof(float));
      res_length = res_lengthT;
      for( uint32_t i = 0; i < res_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_res;
      u_st_res.base = 0;
      u_st_res.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_res.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_res.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_res.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_res = u_st_res.real;
      offset += sizeof(this->st_res);
        memcpy( &(this->res[i]), &(this->st_res), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return FLOATS_ARRAY; };
    const char * getMD5(){ return "718f5814793a2786e65a61ce661074f4"; };

  };

  class Floats_array {
    public:
    typedef Floats_arrayRequest Request;
    typedef Floats_arrayResponse Response;
  };

}
#endif
