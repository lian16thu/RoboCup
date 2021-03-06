// Generated by gencpp from file localization/MeanPoseConfStamped.msg
// DO NOT EDIT!


#ifndef LOCALIZATION_MESSAGE_MEANPOSECONFSTAMPED_H
#define LOCALIZATION_MESSAGE_MEANPOSECONFSTAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose2D.h>

namespace localization
{
template <class ContainerAllocator>
struct MeanPoseConfStamped_
{
  typedef MeanPoseConfStamped_<ContainerAllocator> Type;

  MeanPoseConfStamped_()
    : header()
    , robotPose()
    , robotPoseConfidence(0.0)  {
    }
  MeanPoseConfStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , robotPose(_alloc)
    , robotPoseConfidence(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _robotPose_type;
  _robotPose_type robotPose;

   typedef float _robotPoseConfidence_type;
  _robotPoseConfidence_type robotPoseConfidence;





  typedef boost::shared_ptr< ::localization::MeanPoseConfStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::localization::MeanPoseConfStamped_<ContainerAllocator> const> ConstPtr;

}; // struct MeanPoseConfStamped_

typedef ::localization::MeanPoseConfStamped_<std::allocator<void> > MeanPoseConfStamped;

typedef boost::shared_ptr< ::localization::MeanPoseConfStamped > MeanPoseConfStampedPtr;
typedef boost::shared_ptr< ::localization::MeanPoseConfStamped const> MeanPoseConfStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::localization::MeanPoseConfStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::localization::MeanPoseConfStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace localization

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'tf': ['/opt/ros/kinetic/share/tf/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'localization': ['/home/lian/robot_ws/src/localization/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::localization::MeanPoseConfStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::localization::MeanPoseConfStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::localization::MeanPoseConfStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::localization::MeanPoseConfStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::localization::MeanPoseConfStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::localization::MeanPoseConfStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::localization::MeanPoseConfStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "87d282eb9ec7ff5b36abcb25698c079d";
  }

  static const char* value(const ::localization::MeanPoseConfStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x87d282eb9ec7ff5bULL;
  static const uint64_t static_value2 = 0x36abcb25698c079dULL;
};

template<class ContainerAllocator>
struct DataType< ::localization::MeanPoseConfStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "localization/MeanPoseConfStamped";
  }

  static const char* value(const ::localization::MeanPoseConfStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::localization::MeanPoseConfStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header 				# for time stamp\n\
geometry_msgs/Pose2D robotPose        		# Pose of the robot according to the particle filter localization\n\
float32 robotPoseConfidence\n\
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

  static const char* value(const ::localization::MeanPoseConfStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::localization::MeanPoseConfStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.robotPose);
      stream.next(m.robotPoseConfidence);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MeanPoseConfStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::localization::MeanPoseConfStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::localization::MeanPoseConfStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "robotPose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.robotPose);
    s << indent << "robotPoseConfidence: ";
    Printer<float>::stream(s, indent + "  ", v.robotPoseConfidence);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LOCALIZATION_MESSAGE_MEANPOSECONFSTAMPED_H
