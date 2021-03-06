// Generated by gencpp from file roscpp_tutorials/Uncertainty.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_UNCERTAINTY_H
#define ROSCPP_TUTORIALS_MESSAGE_UNCERTAINTY_H


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
struct Uncertainty_
{
  typedef Uncertainty_<ContainerAllocator> Type;

  Uncertainty_()
    : position_std_dev()
    , velocity_std_dev()
    , orientation_std_dev()  {
      position_std_dev.assign(0.0);

      velocity_std_dev.assign(0.0);

      orientation_std_dev.assign(0.0);
  }
  Uncertainty_(const ContainerAllocator& _alloc)
    : position_std_dev()
    , velocity_std_dev()
    , orientation_std_dev()  {
  (void)_alloc;
      position_std_dev.assign(0.0);

      velocity_std_dev.assign(0.0);

      orientation_std_dev.assign(0.0);
  }



   typedef boost::array<float, 3>  _position_std_dev_type;
  _position_std_dev_type position_std_dev;

   typedef boost::array<float, 3>  _velocity_std_dev_type;
  _velocity_std_dev_type velocity_std_dev;

   typedef boost::array<float, 3>  _orientation_std_dev_type;
  _orientation_std_dev_type orientation_std_dev;





  typedef boost::shared_ptr< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> const> ConstPtr;

}; // struct Uncertainty_

typedef ::roscpp_tutorials::Uncertainty_<std::allocator<void> > Uncertainty;

typedef boost::shared_ptr< ::roscpp_tutorials::Uncertainty > UncertaintyPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::Uncertainty const> UncertaintyConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::Uncertainty_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3f5f772620f0fcfdf53833b907a04cda";
  }

  static const char* value(const ::roscpp_tutorials::Uncertainty_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3f5f772620f0fcfdULL;
  static const uint64_t static_value2 = 0xf53833b907a04cdaULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/Uncertainty";
  }

  static const char* value(const ::roscpp_tutorials::Uncertainty_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[3] position_std_dev\n\
float32[3] velocity_std_dev\n\
float32[3] orientation_std_dev\n\
";
  }

  static const char* value(const ::roscpp_tutorials::Uncertainty_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position_std_dev);
      stream.next(m.velocity_std_dev);
      stream.next(m.orientation_std_dev);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Uncertainty_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::Uncertainty_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::Uncertainty_<ContainerAllocator>& v)
  {
    s << indent << "position_std_dev[]" << std::endl;
    for (size_t i = 0; i < v.position_std_dev.size(); ++i)
    {
      s << indent << "  position_std_dev[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.position_std_dev[i]);
    }
    s << indent << "velocity_std_dev[]" << std::endl;
    for (size_t i = 0; i < v.velocity_std_dev.size(); ++i)
    {
      s << indent << "  velocity_std_dev[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.velocity_std_dev[i]);
    }
    s << indent << "orientation_std_dev[]" << std::endl;
    for (size_t i = 0; i < v.orientation_std_dev.size(); ++i)
    {
      s << indent << "  orientation_std_dev[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.orientation_std_dev[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_UNCERTAINTY_H
