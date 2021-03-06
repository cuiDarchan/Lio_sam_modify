// Generated by gencpp from file roscpp_tutorials/WiperCmd.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_WIPERCMD_H
#define ROSCPP_TUTORIALS_MESSAGE_WIPERCMD_H


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
struct WiperCmd_
{
  typedef WiperCmd_<ContainerAllocator> Type;

  WiperCmd_()
    : wiper_status(0)
    , wiper_speed(0)  {
    }
  WiperCmd_(const ContainerAllocator& _alloc)
    : wiper_status(0)
    , wiper_speed(0)  {
  (void)_alloc;
    }



   typedef uint16_t _wiper_status_type;
  _wiper_status_type wiper_status;

   typedef uint16_t _wiper_speed_type;
  _wiper_speed_type wiper_speed;





  typedef boost::shared_ptr< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> const> ConstPtr;

}; // struct WiperCmd_

typedef ::roscpp_tutorials::WiperCmd_<std::allocator<void> > WiperCmd;

typedef boost::shared_ptr< ::roscpp_tutorials::WiperCmd > WiperCmdPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::WiperCmd const> WiperCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::WiperCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b79ca283455c2ce2df362bd2fe7c94a1";
  }

  static const char* value(const ::roscpp_tutorials::WiperCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb79ca283455c2ce2ULL;
  static const uint64_t static_value2 = 0xdf362bd2fe7c94a1ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/WiperCmd";
  }

  static const char* value(const ::roscpp_tutorials::WiperCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16 wiper_status    \n\
uint16 wiper_speed\n\
";
  }

  static const char* value(const ::roscpp_tutorials::WiperCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.wiper_status);
      stream.next(m.wiper_speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WiperCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::WiperCmd_<ContainerAllocator>& v)
  {
    s << indent << "wiper_status: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.wiper_status);
    s << indent << "wiper_speed: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.wiper_speed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_WIPERCMD_H
