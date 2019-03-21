// Generated by gencpp from file fiducial_msgs/FiducialTransformArray.msg
// DO NOT EDIT!


#ifndef FIDUCIAL_MSGS_MESSAGE_FIDUCIALTRANSFORMARRAY_H
#define FIDUCIAL_MSGS_MESSAGE_FIDUCIALTRANSFORMARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <fiducial_msgs/FiducialTransform.h>

namespace fiducial_msgs
{
template <class ContainerAllocator>
struct FiducialTransformArray_
{
  typedef FiducialTransformArray_<ContainerAllocator> Type;

  FiducialTransformArray_()
    : header()
    , image_seq(0)
    , transforms()  {
    }
  FiducialTransformArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , image_seq(0)
    , transforms(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _image_seq_type;
  _image_seq_type image_seq;

   typedef std::vector< ::fiducial_msgs::FiducialTransform_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::fiducial_msgs::FiducialTransform_<ContainerAllocator> >::other >  _transforms_type;
  _transforms_type transforms;





  typedef boost::shared_ptr< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> const> ConstPtr;

}; // struct FiducialTransformArray_

typedef ::fiducial_msgs::FiducialTransformArray_<std::allocator<void> > FiducialTransformArray;

typedef boost::shared_ptr< ::fiducial_msgs::FiducialTransformArray > FiducialTransformArrayPtr;
typedef boost::shared_ptr< ::fiducial_msgs::FiducialTransformArray const> FiducialTransformArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace fiducial_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'fiducial_msgs': ['/home/lian/robot_ws/src/fiducial_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a9391b25df2859cb1ae6fa6ee45ef075";
  }

  static const char* value(const ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa9391b25df2859cbULL;
  static const uint64_t static_value2 = 0x1ae6fa6ee45ef075ULL;
};

template<class ContainerAllocator>
struct DataType< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fiducial_msgs/FiducialTransformArray";
  }

  static const char* value(const ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return " # A set of camera to fiducial transform with supporting data corresponding\n\
 # to an image\n\
 Header header\n\
 int32 image_seq\n\
 FiducialTransform[] transforms \n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: fiducial_msgs/FiducialTransform\n\
 # A camera to fiducial transform with supporting data\n\
 int32 fiducial_id\n\
 geometry_msgs/Transform transform\n\
 float64 image_error\n\
 float64 object_error\n\
 float64 fiducial_area\n\
================================================================================\n\
MSG: geometry_msgs/Transform\n\
# This represents the transform between two coordinate frames in free space.\n\
\n\
Vector3 translation\n\
Quaternion rotation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.image_seq);
      stream.next(m.transforms);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FiducialTransformArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fiducial_msgs::FiducialTransformArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "image_seq: ";
    Printer<int32_t>::stream(s, indent + "  ", v.image_seq);
    s << indent << "transforms[]" << std::endl;
    for (size_t i = 0; i < v.transforms.size(); ++i)
    {
      s << indent << "  transforms[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::fiducial_msgs::FiducialTransform_<ContainerAllocator> >::stream(s, indent + "    ", v.transforms[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FIDUCIAL_MSGS_MESSAGE_FIDUCIALTRANSFORMARRAY_H
