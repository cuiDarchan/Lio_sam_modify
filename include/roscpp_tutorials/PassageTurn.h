// Generated by gencpp from file roscpp_tutorials/PassageTurn.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_PASSAGETURN_H
#define ROSCPP_TUTORIALS_MESSAGE_PASSAGETURN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct PassageTurn_
{
  typedef PassageTurn_<ContainerAllocator> Type;

  PassageTurn_()
    : turn_type(0)
    , start_s(0.0)
    , end_s(0.0)  {
    }
  PassageTurn_(const ContainerAllocator& _alloc)
    : turn_type(0)
    , start_s(0.0)
    , end_s(0.0)  {
  (void)_alloc;
    }



   typedef uint16_t _turn_type_type;
  _turn_type_type turn_type;

   typedef float _start_s_type;
  _start_s_type start_s;

   typedef float _end_s_type;
  _end_s_type end_s;





  typedef boost::shared_ptr< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> const> ConstPtr;

}; // struct PassageTurn_

typedef ::roscpp_tutorials::PassageTurn_<std::allocator<void> > PassageTurn;

typedef boost::shared_ptr< ::roscpp_tutorials::PassageTurn > PassageTurnPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::PassageTurn const> PassageTurnConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::PassageTurn_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace roscpp_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'roscpp_tutorials': ['/home/cui-dell/catkin_ws/src/roscpp_tutorials/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "da6b7dea313942c028d3fdaef05eeb7f";
  }

  static const char* value(const ::roscpp_tutorials::PassageTurn_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xda6b7dea313942c0ULL;
  static const uint64_t static_value2 = 0x28d3fdaef05eeb7fULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/PassageTurn";
  }

  static const char* value(const ::roscpp_tutorials::PassageTurn_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "    uint16 turn_type\n\
    float32 start_s\n\
    float32 end_s\n\
";
  }

  static const char* value(const ::roscpp_tutorials::PassageTurn_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.turn_type);
      stream.next(m.start_s);
      stream.next(m.end_s);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PassageTurn_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::PassageTurn_<ContainerAllocator>& v)
  {
    s << indent << "turn_type: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.turn_type);
    s << indent << "start_s: ";
    Printer<float>::stream(s, indent + "  ", v.start_s);
    s << indent << "end_s: ";
    Printer<float>::stream(s, indent + "  ", v.end_s);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_PASSAGETURN_H