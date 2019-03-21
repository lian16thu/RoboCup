// Generated by gencpp from file localization/GoalpostsDetected.msg
// DO NOT EDIT!


#ifndef LOCALIZATION_MESSAGE_GOALPOSTSDETECTED_H
#define LOCALIZATION_MESSAGE_GOALPOSTSDETECTED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose2D.h>

namespace localization
{
template <class ContainerAllocator>
struct GoalpostsDetected_
{
  typedef GoalpostsDetected_<ContainerAllocator> Type;

  GoalpostsDetected_()
    : pose()
    , type(0)
    , confidence(0.0)  {
    }
  GoalpostsDetected_(const ContainerAllocator& _alloc)
    : pose(_alloc)
    , type(0)
    , confidence(0.0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef uint8_t _type_type;
  _type_type type;

   typedef float _confidence_type;
  _confidence_type confidence;





  typedef boost::shared_ptr< ::localization::GoalpostsDetected_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::localization::GoalpostsDetected_<ContainerAllocator> const> ConstPtr;

}; // struct GoalpostsDetected_

typedef ::localization::GoalpostsDetected_<std::allocator<void> > GoalpostsDetected;

typedef boost::shared_ptr< ::localization::GoalpostsDetected > GoalpostsDetectedPtr;
typedef boost::shared_ptr< ::localization::GoalpostsDetected const> GoalpostsDetectedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::localization::GoalpostsDetected_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::localization::GoalpostsDetected_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::localization::GoalpostsDetected_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::localization::GoalpostsDetected_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::localization::GoalpostsDetected_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::localization::GoalpostsDetected_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::localization::GoalpostsDetected_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::localization::GoalpostsDetected_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::localization::GoalpostsDetected_<ContainerAllocator> >
{
  static const char* value()
  {
    return "45117ab335e578bb32c6e4a315f61f4d";
  }

  static const char* value(const ::localization::GoalpostsDetected_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x45117ab335e578bbULL;
  static const uint64_t static_value2 = 0x32c6e4a315f61f4dULL;
};

template<class ContainerAllocator>
struct DataType< ::localization::GoalpostsDetected_<ContainerAllocator> >
{
  static const char* value()
  {
    return "localization/GoalpostsDetected";
  }

  static const char* value(const ::localization::GoalpostsDetected_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::localization::GoalpostsDetected_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Pose2D pose        # Pose\n\
uint8 type                       # Type (see field_model::WorldObject::Type)\n\
float32 confidence               # confidence 0..1\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# Deprecated\n\
# Please use the full 3D pose.\n\
\n\
# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n\
\n\
# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n\
\n\
\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const ::localization::GoalpostsDetected_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::localization::GoalpostsDetected_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
      stream.next(m.type);
      stream.next(m.confidence);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoalpostsDetected_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::localization::GoalpostsDetected_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::localization::GoalpostsDetected_<ContainerAllocator>& v)
  {
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "confidence: ";
    Printer<float>::stream(s, indent + "  ", v.confidence);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LOCALIZATION_MESSAGE_GOALPOSTSDETECTED_H