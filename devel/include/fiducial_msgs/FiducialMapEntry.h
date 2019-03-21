// Generated by gencpp from file fiducial_msgs/FiducialMapEntry.msg
// DO NOT EDIT!


#ifndef FIDUCIAL_MSGS_MESSAGE_FIDUCIALMAPENTRY_H
#define FIDUCIAL_MSGS_MESSAGE_FIDUCIALMAPENTRY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace fiducial_msgs
{
template <class ContainerAllocator>
struct FiducialMapEntry_
{
  typedef FiducialMapEntry_<ContainerAllocator> Type;

  FiducialMapEntry_()
    : fiducial_id(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , rx(0.0)
    , ry(0.0)
    , rz(0.0)  {
    }
  FiducialMapEntry_(const ContainerAllocator& _alloc)
    : fiducial_id(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , rx(0.0)
    , ry(0.0)
    , rz(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _fiducial_id_type;
  _fiducial_id_type fiducial_id;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _rx_type;
  _rx_type rx;

   typedef double _ry_type;
  _ry_type ry;

   typedef double _rz_type;
  _rz_type rz;





  typedef boost::shared_ptr< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> const> ConstPtr;

}; // struct FiducialMapEntry_

typedef ::fiducial_msgs::FiducialMapEntry_<std::allocator<void> > FiducialMapEntry;

typedef boost::shared_ptr< ::fiducial_msgs::FiducialMapEntry > FiducialMapEntryPtr;
typedef boost::shared_ptr< ::fiducial_msgs::FiducialMapEntry const> FiducialMapEntryConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace fiducial_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'fiducial_msgs': ['/home/lian/robot_ws/src/fiducial_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2dc3e2ac5967f3a2c19627a1fc1d7dcc";
  }

  static const char* value(const ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2dc3e2ac5967f3a2ULL;
  static const uint64_t static_value2 = 0xc19627a1fc1d7dccULL;
};

template<class ContainerAllocator>
struct DataType< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fiducial_msgs/FiducialMapEntry";
  }

  static const char* value(const ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# pose of a fiducial\n\
int32 fiducial_id\n\
# translation\n\
float64 x\n\
float64 y\n\
float64 z\n\
# rotation\n\
float64 rx\n\
float64 ry\n\
float64 rz\n\
\n\
";
  }

  static const char* value(const ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.fiducial_id);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.rx);
      stream.next(m.ry);
      stream.next(m.rz);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FiducialMapEntry_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator>& v)
  {
    s << indent << "fiducial_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.fiducial_id);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "rx: ";
    Printer<double>::stream(s, indent + "  ", v.rx);
    s << indent << "ry: ";
    Printer<double>::stream(s, indent + "  ", v.ry);
    s << indent << "rz: ";
    Printer<double>::stream(s, indent + "  ", v.rz);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FIDUCIAL_MSGS_MESSAGE_FIDUCIALMAPENTRY_H
