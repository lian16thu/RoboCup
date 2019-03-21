// Generated by gencpp from file vision/LinesLandmarks.msg
// DO NOT EDIT!


#ifndef VISION_MESSAGE_LINESLANDMARKS_H
#define VISION_MESSAGE_LINESLANDMARKS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <vision/Line.h>
#include <vision/Landmark.h>

namespace vision
{
template <class ContainerAllocator>
struct LinesLandmarks_
{
  typedef LinesLandmarks_<ContainerAllocator> Type;

  LinesLandmarks_()
    : lines()
    , landmarks()  {
    }
  LinesLandmarks_(const ContainerAllocator& _alloc)
    : lines(_alloc)
    , landmarks(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::vision::Line_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::vision::Line_<ContainerAllocator> >::other >  _lines_type;
  _lines_type lines;

   typedef std::vector< ::vision::Landmark_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::vision::Landmark_<ContainerAllocator> >::other >  _landmarks_type;
  _landmarks_type landmarks;





  typedef boost::shared_ptr< ::vision::LinesLandmarks_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vision::LinesLandmarks_<ContainerAllocator> const> ConstPtr;

}; // struct LinesLandmarks_

typedef ::vision::LinesLandmarks_<std::allocator<void> > LinesLandmarks;

typedef boost::shared_ptr< ::vision::LinesLandmarks > LinesLandmarksPtr;
typedef boost::shared_ptr< ::vision::LinesLandmarks const> LinesLandmarksConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vision::LinesLandmarks_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vision::LinesLandmarks_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace vision

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'vision': ['/home/lian/robot_ws/src/vision/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::vision::LinesLandmarks_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision::LinesLandmarks_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vision::LinesLandmarks_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vision::LinesLandmarks_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision::LinesLandmarks_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision::LinesLandmarks_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vision::LinesLandmarks_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fb66f810489cc88ff8dd15c871c9425d";
  }

  static const char* value(const ::vision::LinesLandmarks_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfb66f810489cc88fULL;
  static const uint64_t static_value2 = 0xf8dd15c871c9425dULL;
};

template<class ContainerAllocator>
struct DataType< ::vision::LinesLandmarks_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vision/LinesLandmarks";
  }

  static const char* value(const ::vision::LinesLandmarks_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vision::LinesLandmarks_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Line[] lines\n\
Landmark[] landmarks\n\
\n\
================================================================================\n\
MSG: vision/Line\n\
float32 x1 #line start\n\
float32 y1\n\
float32 x2 #line end\n\
float32 y2\n\
\n\
================================================================================\n\
MSG: vision/Landmark\n\
geometry_msgs/Pose2D pose        # Pose\n\
uint8 type                       # Type (see localization::field_model::WorldObject::Type)\n\
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

  static const char* value(const ::vision::LinesLandmarks_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vision::LinesLandmarks_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.lines);
      stream.next(m.landmarks);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LinesLandmarks_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vision::LinesLandmarks_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vision::LinesLandmarks_<ContainerAllocator>& v)
  {
    s << indent << "lines[]" << std::endl;
    for (size_t i = 0; i < v.lines.size(); ++i)
    {
      s << indent << "  lines[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::vision::Line_<ContainerAllocator> >::stream(s, indent + "    ", v.lines[i]);
    }
    s << indent << "landmarks[]" << std::endl;
    for (size_t i = 0; i < v.landmarks.size(); ++i)
    {
      s << indent << "  landmarks[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::vision::Landmark_<ContainerAllocator> >::stream(s, indent + "    ", v.landmarks[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISION_MESSAGE_LINESLANDMARKS_H
