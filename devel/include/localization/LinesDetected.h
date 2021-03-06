// Generated by gencpp from file localization/LinesDetected.msg
// DO NOT EDIT!


#ifndef LOCALIZATION_MESSAGE_LINESDETECTED_H
#define LOCALIZATION_MESSAGE_LINESDETECTED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace localization
{
template <class ContainerAllocator>
struct LinesDetected_
{
  typedef LinesDetected_<ContainerAllocator> Type;

  LinesDetected_()
    : x1(0.0)
    , y1(0.0)
    , x2(0.0)
    , y2(0.0)  {
    }
  LinesDetected_(const ContainerAllocator& _alloc)
    : x1(0.0)
    , y1(0.0)
    , x2(0.0)
    , y2(0.0)  {
  (void)_alloc;
    }



   typedef float _x1_type;
  _x1_type x1;

   typedef float _y1_type;
  _y1_type y1;

   typedef float _x2_type;
  _x2_type x2;

   typedef float _y2_type;
  _y2_type y2;





  typedef boost::shared_ptr< ::localization::LinesDetected_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::localization::LinesDetected_<ContainerAllocator> const> ConstPtr;

}; // struct LinesDetected_

typedef ::localization::LinesDetected_<std::allocator<void> > LinesDetected;

typedef boost::shared_ptr< ::localization::LinesDetected > LinesDetectedPtr;
typedef boost::shared_ptr< ::localization::LinesDetected const> LinesDetectedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::localization::LinesDetected_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::localization::LinesDetected_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace localization

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'tf': ['/opt/ros/kinetic/share/tf/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'localization': ['/home/lian/robot_ws/src/localization/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::localization::LinesDetected_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::localization::LinesDetected_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::localization::LinesDetected_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::localization::LinesDetected_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::localization::LinesDetected_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::localization::LinesDetected_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::localization::LinesDetected_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1d74979d8119401281d48677f845994f";
  }

  static const char* value(const ::localization::LinesDetected_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1d74979d81194012ULL;
  static const uint64_t static_value2 = 0x81d48677f845994fULL;
};

template<class ContainerAllocator>
struct DataType< ::localization::LinesDetected_<ContainerAllocator> >
{
  static const char* value()
  {
    return "localization/LinesDetected";
  }

  static const char* value(const ::localization::LinesDetected_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::localization::LinesDetected_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x1\n\
float32 y1\n\
float32 x2\n\
float32 y2\n\
";
  }

  static const char* value(const ::localization::LinesDetected_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::localization::LinesDetected_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x1);
      stream.next(m.y1);
      stream.next(m.x2);
      stream.next(m.y2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LinesDetected_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::localization::LinesDetected_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::localization::LinesDetected_<ContainerAllocator>& v)
  {
    s << indent << "x1: ";
    Printer<float>::stream(s, indent + "  ", v.x1);
    s << indent << "y1: ";
    Printer<float>::stream(s, indent + "  ", v.y1);
    s << indent << "x2: ";
    Printer<float>::stream(s, indent + "  ", v.x2);
    s << indent << "y2: ";
    Printer<float>::stream(s, indent + "  ", v.y2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LOCALIZATION_MESSAGE_LINESDETECTED_H
