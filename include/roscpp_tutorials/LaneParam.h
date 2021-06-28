// Generated by gencpp from file roscpp_tutorials/LaneParam.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_LANEPARAM_H
#define ROSCPP_TUTORIALS_MESSAGE_LANEPARAM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/Point3d.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct LaneParam_
{
  typedef LaneParam_<ContainerAllocator> Type;

  LaneParam_()
    : cx_quality(0.0)
    , start(0.0)
    , end(0.0)
    , c0(0.0)
    , c1(0.0)
    , c2(0.0)
    , c3(0.0)
    , samples()
    , samples_confidence(0.0)  {
    }
  LaneParam_(const ContainerAllocator& _alloc)
    : cx_quality(0.0)
    , start(0.0)
    , end(0.0)
    , c0(0.0)
    , c1(0.0)
    , c2(0.0)
    , c3(0.0)
    , samples(_alloc)
    , samples_confidence(0.0)  {
  (void)_alloc;
    }



   typedef float _cx_quality_type;
  _cx_quality_type cx_quality;

   typedef float _start_type;
  _start_type start;

   typedef float _end_type;
  _end_type end;

   typedef float _c0_type;
  _c0_type c0;

   typedef float _c1_type;
  _c1_type c1;

   typedef float _c2_type;
  _c2_type c2;

   typedef float _c3_type;
  _c3_type c3;

   typedef std::vector< ::roscpp_tutorials::Point3d_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::Point3d_<ContainerAllocator> >::other >  _samples_type;
  _samples_type samples;

   typedef float _samples_confidence_type;
  _samples_confidence_type samples_confidence;





  typedef boost::shared_ptr< ::roscpp_tutorials::LaneParam_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::LaneParam_<ContainerAllocator> const> ConstPtr;

}; // struct LaneParam_

typedef ::roscpp_tutorials::LaneParam_<std::allocator<void> > LaneParam;

typedef boost::shared_ptr< ::roscpp_tutorials::LaneParam > LaneParamPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::LaneParam const> LaneParamConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::LaneParam_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::LaneParam_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::LaneParam_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::LaneParam_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::LaneParam_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::LaneParam_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::LaneParam_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::LaneParam_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::LaneParam_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c035ee86362c8cbad7778401cd7ff8c0";
  }

  static const char* value(const ::roscpp_tutorials::LaneParam_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc035ee86362c8cbaULL;
  static const uint64_t static_value2 = 0xd7778401cd7ff8c0ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::LaneParam_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/LaneParam";
  }

  static const char* value(const ::roscpp_tutorials::LaneParam_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::LaneParam_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 cx_quality\n\
    \n\
float32 start\n\
    \n\
float32 end\n\
    \n\
float32 c0\n\
    \n\
float32 c1\n\
    \n\
float32 c2\n\
    \n\
float32 c3\n\
\n\
Point3d[] samples\n\
    \n\
float32 samples_confidence\n\
================================================================================\n\
MSG: roscpp_tutorials/Point3d\n\
	float32 x  # in meters or m/s\n\
	float32 y  # in meters or m/s\n\
	float32 z  # height in meters or m/s\n\
";
  }

  static const char* value(const ::roscpp_tutorials::LaneParam_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::LaneParam_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cx_quality);
      stream.next(m.start);
      stream.next(m.end);
      stream.next(m.c0);
      stream.next(m.c1);
      stream.next(m.c2);
      stream.next(m.c3);
      stream.next(m.samples);
      stream.next(m.samples_confidence);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LaneParam_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::LaneParam_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::LaneParam_<ContainerAllocator>& v)
  {
    s << indent << "cx_quality: ";
    Printer<float>::stream(s, indent + "  ", v.cx_quality);
    s << indent << "start: ";
    Printer<float>::stream(s, indent + "  ", v.start);
    s << indent << "end: ";
    Printer<float>::stream(s, indent + "  ", v.end);
    s << indent << "c0: ";
    Printer<float>::stream(s, indent + "  ", v.c0);
    s << indent << "c1: ";
    Printer<float>::stream(s, indent + "  ", v.c1);
    s << indent << "c2: ";
    Printer<float>::stream(s, indent + "  ", v.c2);
    s << indent << "c3: ";
    Printer<float>::stream(s, indent + "  ", v.c3);
    s << indent << "samples[]" << std::endl;
    for (size_t i = 0; i < v.samples.size(); ++i)
    {
      s << indent << "  samples[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::Point3d_<ContainerAllocator> >::stream(s, indent + "    ", v.samples[i]);
    }
    s << indent << "samples_confidence: ";
    Printer<float>::stream(s, indent + "  ", v.samples_confidence);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_LANEPARAM_H