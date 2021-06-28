// Generated by gencpp from file roscpp_tutorials/TrajectoryPoint.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_TRAJECTORYPOINT_H
#define ROSCPP_TUTORIALS_MESSAGE_TRAJECTORYPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/PathPoint.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct TrajectoryPoint_
{
  typedef TrajectoryPoint_<ContainerAllocator> Type;

  TrajectoryPoint_()
    : path_point()
    , v(0.0)
    , a(0.0)
    , relative_time(0.0)
    , da(0.0)  {
    }
  TrajectoryPoint_(const ContainerAllocator& _alloc)
    : path_point(_alloc)
    , v(0.0)
    , a(0.0)
    , relative_time(0.0)
    , da(0.0)  {
  (void)_alloc;
    }



   typedef  ::roscpp_tutorials::PathPoint_<ContainerAllocator>  _path_point_type;
  _path_point_type path_point;

   typedef float _v_type;
  _v_type v;

   typedef float _a_type;
  _a_type a;

   typedef float _relative_time_type;
  _relative_time_type relative_time;

   typedef float _da_type;
  _da_type da;





  typedef boost::shared_ptr< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> const> ConstPtr;

}; // struct TrajectoryPoint_

typedef ::roscpp_tutorials::TrajectoryPoint_<std::allocator<void> > TrajectoryPoint;

typedef boost::shared_ptr< ::roscpp_tutorials::TrajectoryPoint > TrajectoryPointPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::TrajectoryPoint const> TrajectoryPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e3124e47ddd80228a2a902faf95c0596";
  }

  static const char* value(const ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe3124e47ddd80228ULL;
  static const uint64_t static_value2 = 0xa2a902faf95c0596ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/TrajectoryPoint";
  }

  static const char* value(const ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "PathPoint path_point\n\
float32 v\n\
float32 a\n\
float32 relative_time\n\
float32 da\n\
================================================================================\n\
MSG: roscpp_tutorials/PathPoint\n\
Point3d position\n\
float32 theta\n\
float32 kappa\n\
float32 s\n\
float32 dkappa\n\
float32 ddkappa\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/Point3d\n\
	float32 x  # in meters or m/s\n\
	float32 y  # in meters or m/s\n\
	float32 z  # height in meters or m/s\n\
";
  }

  static const char* value(const ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.path_point);
      stream.next(m.v);
      stream.next(m.a);
      stream.next(m.relative_time);
      stream.next(m.da);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrajectoryPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::TrajectoryPoint_<ContainerAllocator>& v)
  {
    s << indent << "path_point: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::PathPoint_<ContainerAllocator> >::stream(s, indent + "  ", v.path_point);
    s << indent << "v: ";
    Printer<float>::stream(s, indent + "  ", v.v);
    s << indent << "a: ";
    Printer<float>::stream(s, indent + "  ", v.a);
    s << indent << "relative_time: ";
    Printer<float>::stream(s, indent + "  ", v.relative_time);
    s << indent << "da: ";
    Printer<float>::stream(s, indent + "  ", v.da);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_TRAJECTORYPOINT_H