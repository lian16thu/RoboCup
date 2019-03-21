// Generated by gencpp from file decision/GoalData.msg
// DO NOT EDIT!


#ifndef DECISION_MESSAGE_GOALDATA_H
#define DECISION_MESSAGE_GOALDATA_H


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
struct GoalData_
{
  typedef GoalData_<ContainerAllocator> Type;

  GoalData_()
    : goal(0)
    , leftx(0.0)
    , lefty(0.0)
    , rightx(0.0)
    , righty(0.0)  {
    }
  GoalData_(const ContainerAllocator& _alloc)
    : goal(0)
    , leftx(0.0)
    , lefty(0.0)
    , rightx(0.0)
    , righty(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _goal_type;
  _goal_type goal;

   typedef float _leftx_type;
  _leftx_type leftx;

   typedef float _lefty_type;
  _lefty_type lefty;

   typedef float _rightx_type;
  _rightx_type rightx;

   typedef float _righty_type;
  _righty_type righty;





  typedef boost::shared_ptr< ::decision::GoalData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::decision::GoalData_<ContainerAllocator> const> ConstPtr;

}; // struct GoalData_

typedef ::decision::GoalData_<std::allocator<void> > GoalData;

typedef boost::shared_ptr< ::decision::GoalData > GoalDataPtr;
typedef boost::shared_ptr< ::decision::GoalData const> GoalDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::decision::GoalData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::decision::GoalData_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::decision::GoalData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::decision::GoalData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::decision::GoalData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::decision::GoalData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::decision::GoalData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::decision::GoalData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::decision::GoalData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "657b4b89e12eaea48f7336974eb25b11";
  }

  static const char* value(const ::decision::GoalData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x657b4b89e12eaea4ULL;
  static const uint64_t static_value2 = 0x8f7336974eb25b11ULL;
};

template<class ContainerAllocator>
struct DataType< ::decision::GoalData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "decision/GoalData";
  }

  static const char* value(const ::decision::GoalData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::decision::GoalData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 goal\n\
float32 leftx\n\
float32 lefty\n\
float32 rightx\n\
float32 righty\n\
\n\
";
  }

  static const char* value(const ::decision::GoalData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::decision::GoalData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.goal);
      stream.next(m.leftx);
      stream.next(m.lefty);
      stream.next(m.rightx);
      stream.next(m.righty);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoalData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::decision::GoalData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::decision::GoalData_<ContainerAllocator>& v)
  {
    s << indent << "goal: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.goal);
    s << indent << "leftx: ";
    Printer<float>::stream(s, indent + "  ", v.leftx);
    s << indent << "lefty: ";
    Printer<float>::stream(s, indent + "  ", v.lefty);
    s << indent << "rightx: ";
    Printer<float>::stream(s, indent + "  ", v.rightx);
    s << indent << "righty: ";
    Printer<float>::stream(s, indent + "  ", v.righty);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DECISION_MESSAGE_GOALDATA_H