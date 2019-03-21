// Generated by gencpp from file decision/gameControl.msg
// DO NOT EDIT!


#ifndef DECISION_MESSAGE_GAMECONTROL_H
#define DECISION_MESSAGE_GAMECONTROL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace decision
{
template <class ContainerAllocator>
struct gameControl_
{
  typedef gameControl_<ContainerAllocator> Type;

  gameControl_()
    : state(0)
    , secsRemaining(0)  {
    }
  gameControl_(const ContainerAllocator& _alloc)
    : state(0)
    , secsRemaining(0)  {
  (void)_alloc;
    }



   typedef int32_t _state_type;
  _state_type state;

   typedef int32_t _secsRemaining_type;
  _secsRemaining_type secsRemaining;





  typedef boost::shared_ptr< ::decision::gameControl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::decision::gameControl_<ContainerAllocator> const> ConstPtr;

}; // struct gameControl_

typedef ::decision::gameControl_<std::allocator<void> > gameControl;

typedef boost::shared_ptr< ::decision::gameControl > gameControlPtr;
typedef boost::shared_ptr< ::decision::gameControl const> gameControlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::decision::gameControl_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::decision::gameControl_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace decision

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'decision': ['/home/lian/robot_ws/src/decision/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::decision::gameControl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::decision::gameControl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::decision::gameControl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::decision::gameControl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::decision::gameControl_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::decision::gameControl_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::decision::gameControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f8fd4e28d3da76570da7ab09212fb90";
  }

  static const char* value(const ::decision::gameControl_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f8fd4e28d3da765ULL;
  static const uint64_t static_value2 = 0x70da7ab09212fb90ULL;
};

template<class ContainerAllocator>
struct DataType< ::decision::gameControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "decision/gameControl";
  }

  static const char* value(const ::decision::gameControl_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::decision::gameControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 state\n\
int32 secsRemaining\n\
";
  }

  static const char* value(const ::decision::gameControl_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::decision::gameControl_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
      stream.next(m.secsRemaining);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gameControl_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::decision::gameControl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::decision::gameControl_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<int32_t>::stream(s, indent + "  ", v.state);
    s << indent << "secsRemaining: ";
    Printer<int32_t>::stream(s, indent + "  ", v.secsRemaining);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DECISION_MESSAGE_GAMECONTROL_H
