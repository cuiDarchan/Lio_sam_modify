// Generated by gencpp from file roscpp_tutorials/CurveSign.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_CURVESIGN_H
#define ROSCPP_TUTORIALS_MESSAGE_CURVESIGN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/Curved.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct CurveSign_
{
  typedef CurveSign_<ContainerAllocator> Type;

  CurveSign_()
    : id()
    , type(0)
    , other_type()
    , segments()
    , start_s(0.0)
    , end_s(0.0)  {
    }
  CurveSign_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , type(0)
    , other_type(_alloc)
    , segments(_alloc)
    , start_s(0.0)
    , end_s(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _id_type;
  _id_type id;

   typedef uint16_t _type_type;
  _type_type type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _other_type_type;
  _other_type_type other_type;

   typedef  ::roscpp_tutorials::Curved_<ContainerAllocator>  _segments_type;
  _segments_type segments;

   typedef float _start_s_type;
  _start_s_type start_s;

   typedef float _end_s_type;
  _end_s_type end_s;





  typedef boost::shared_ptr< ::roscpp_tutorials::CurveSign_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::CurveSign_<ContainerAllocator> const> ConstPtr;

}; // struct CurveSign_

typedef ::roscpp_tutorials::CurveSign_<std::allocator<void> > CurveSign;

typedef boost::shared_ptr< ::roscpp_tutorials::CurveSign > CurveSignPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::CurveSign const> CurveSignConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::CurveSign_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace roscpp_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'roscpp_tutorials': ['/home/cui-dell/catkin_ws/src/roscpp_tutorials/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::CurveSign_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::CurveSign_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::CurveSign_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >
{
  static const char* value()
  {
    return "22dae11346b6763b97c8509a0523eaed";
  }

  static const char* value(const ::roscpp_tutorials::CurveSign_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x22dae11346b6763bULL;
  static const uint64_t static_value2 = 0x97c8509a0523eaedULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/CurveSign";
  }

  static const char* value(const ::roscpp_tutorials::CurveSign_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >
{
  static const char* value()
  {
    return "    string id\n\
    uint16 type\n\
    string other_type\n\
    Curved segments\n\
    float32 start_s\n\
    float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/Curved\n\
Vec3d[] points\n\
\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/Vec3d\n\
float32[3] point\n\
\n\
";
  }

  static const char* value(const ::roscpp_tutorials::CurveSign_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.type);
      stream.next(m.other_type);
      stream.next(m.segments);
      stream.next(m.start_s);
      stream.next(m.end_s);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CurveSign_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::CurveSign_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.id);
    s << indent << "type: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.type);
    s << indent << "other_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.other_type);
    s << indent << "segments: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::Curved_<ContainerAllocator> >::stream(s, indent + "  ", v.segments);
    s << indent << "start_s: ";
    Printer<float>::stream(s, indent + "  ", v.start_s);
    s << indent << "end_s: ";
    Printer<float>::stream(s, indent + "  ", v.end_s);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_CURVESIGN_H