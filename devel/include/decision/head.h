// Generated by gencpp from file decision/head.msg
// DO NOT EDIT!


#ifndef DECISION_MESSAGE_HEAD_H
#define DECISION_MESSAGE_HEAD_H

#include <ros/service_traits.h>


#include <decision/headRequest.h>
#include <decision/headResponse.h>


namespace decision
{

struct head
{

typedef headRequest Request;
typedef headResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct head
} // namespace decision


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::decision::head > {
  static const char* value()
  {
    return "e4aff7be97fd6298f4a9b30300ac7316";
  }

  static const char* value(const ::decision::head&) { return value(); }
};

template<>
struct DataType< ::decision::head > {
  static const char* value()
  {
    return "decision/head";
  }

  static const char* value(const ::decision::head&) { return value(); }
};


// service_traits::MD5Sum< ::decision::headRequest> should match 
// service_traits::MD5Sum< ::decision::head > 
template<>
struct MD5Sum< ::decision::headRequest>
{
  static const char* value()
  {
    return MD5Sum< ::decision::head >::value();
  }
  static const char* value(const ::decision::headRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::decision::headRequest> should match 
// service_traits::DataType< ::decision::head > 
template<>
struct DataType< ::decision::headRequest>
{
  static const char* value()
  {
    return DataType< ::decision::head >::value();
  }
  static const char* value(const ::decision::headRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::decision::headResponse> should match 
// service_traits::MD5Sum< ::decision::head > 
template<>
struct MD5Sum< ::decision::headResponse>
{
  static const char* value()
  {
    return MD5Sum< ::decision::head >::value();
  }
  static const char* value(const ::decision::headResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::decision::headResponse> should match 
// service_traits::DataType< ::decision::head > 
template<>
struct DataType< ::decision::headResponse>
{
  static const char* value()
  {
    return DataType< ::decision::head >::value();
  }
  static const char* value(const ::decision::headResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DECISION_MESSAGE_HEAD_H
