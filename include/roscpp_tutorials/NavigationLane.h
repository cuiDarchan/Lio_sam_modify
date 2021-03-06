// Generated by gencpp from file roscpp_tutorials/NavigationLane.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_NAVIGATIONLANE_H
#define ROSCPP_TUTORIALS_MESSAGE_NAVIGATIONLANE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/Curved.h>
#include <roscpp_tutorials/LaneSampleAssociation.h>
#include <roscpp_tutorials/LaneSampleAssociation.h>
#include <roscpp_tutorials/NavigationLaneBoundary.h>
#include <roscpp_tutorials/NavigationLaneBoundary.h>
#include <roscpp_tutorials/CurveSign.h>
#include <roscpp_tutorials/PolygonSign.h>
#include <roscpp_tutorials/Signal.h>
#include <roscpp_tutorials/ParkingSpace.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct NavigationLane_
{
  typedef NavigationLane_<ContainerAllocator> Type;

  NavigationLane_()
    : id()
    , central_curve()
    , left_samples()
    , right_samples()
    , type(0)
    , speed_limit(0.0)
    , left_boundary()
    , right_boundary()
    , predecessor_id()
    , sucessor_id()
    , left_neighbor_id()
    , right_neighbor_id()
    , curve_signs()
    , polygon_signs()
    , signals()
    , parking_spaces()
    , turn(0)  {
    }
  NavigationLane_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , central_curve(_alloc)
    , left_samples(_alloc)
    , right_samples(_alloc)
    , type(0)
    , speed_limit(0.0)
    , left_boundary(_alloc)
    , right_boundary(_alloc)
    , predecessor_id(_alloc)
    , sucessor_id(_alloc)
    , left_neighbor_id(_alloc)
    , right_neighbor_id(_alloc)
    , curve_signs(_alloc)
    , polygon_signs(_alloc)
    , signals(_alloc)
    , parking_spaces(_alloc)
    , turn(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _id_type;
  _id_type id;

   typedef  ::roscpp_tutorials::Curved_<ContainerAllocator>  _central_curve_type;
  _central_curve_type central_curve;

   typedef std::vector< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> >::other >  _left_samples_type;
  _left_samples_type left_samples;

   typedef std::vector< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> >::other >  _right_samples_type;
  _right_samples_type right_samples;

   typedef uint16_t _type_type;
  _type_type type;

   typedef float _speed_limit_type;
  _speed_limit_type speed_limit;

   typedef  ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator>  _left_boundary_type;
  _left_boundary_type left_boundary;

   typedef  ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator>  _right_boundary_type;
  _right_boundary_type right_boundary;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _predecessor_id_type;
  _predecessor_id_type predecessor_id;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _sucessor_id_type;
  _sucessor_id_type sucessor_id;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _left_neighbor_id_type;
  _left_neighbor_id_type left_neighbor_id;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _right_neighbor_id_type;
  _right_neighbor_id_type right_neighbor_id;

   typedef std::vector< ::roscpp_tutorials::CurveSign_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >::other >  _curve_signs_type;
  _curve_signs_type curve_signs;

   typedef std::vector< ::roscpp_tutorials::PolygonSign_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::PolygonSign_<ContainerAllocator> >::other >  _polygon_signs_type;
  _polygon_signs_type polygon_signs;

   typedef std::vector< ::roscpp_tutorials::Signal_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::Signal_<ContainerAllocator> >::other >  _signals_type;
  _signals_type signals;

   typedef std::vector< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >::other >  _parking_spaces_type;
  _parking_spaces_type parking_spaces;

   typedef uint16_t _turn_type;
  _turn_type turn;





  typedef boost::shared_ptr< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> const> ConstPtr;

}; // struct NavigationLane_

typedef ::roscpp_tutorials::NavigationLane_<std::allocator<void> > NavigationLane;

typedef boost::shared_ptr< ::roscpp_tutorials::NavigationLane > NavigationLanePtr;
typedef boost::shared_ptr< ::roscpp_tutorials::NavigationLane const> NavigationLaneConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::NavigationLane_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> >
{
  static const char* value()
  {
    return "294e95725421f9c169b85e9a18099844";
  }

  static const char* value(const ::roscpp_tutorials::NavigationLane_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x294e95725421f9c1ULL;
  static const uint64_t static_value2 = 0x69b85e9a18099844ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/NavigationLane";
  }

  static const char* value(const ::roscpp_tutorials::NavigationLane_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> >
{
  static const char* value()
  {
    return "    string id\n\
    Curved central_curve\n\
    LaneSampleAssociation[] left_samples\n\
    LaneSampleAssociation[] right_samples\n\
    uint16 type\n\
    float32 speed_limit\n\
    NavigationLaneBoundary left_boundary\n\
    NavigationLaneBoundary right_boundary\n\
    string[] predecessor_id\n\
    string[] sucessor_id\n\
    string[] left_neighbor_id\n\
    string[] right_neighbor_id\n\
    CurveSign[] curve_signs\n\
    PolygonSign[] polygon_signs\n\
    Signal[] signals\n\
    ParkingSpace[] parking_spaces\n\
    uint16 turn\n\
================================================================================\n\
MSG: roscpp_tutorials/Curved\n\
Vec3d[] points\n\
\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/Vec3d\n\
float32[3] point\n\
\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/LaneSampleAssociation\n\
float32 start_s\n\
float32 end_s\n\
float32 width\n\
================================================================================\n\
MSG: roscpp_tutorials/NavigationLaneBoundary\n\
LaneBoundaryType[] type\n\
Curved segments\n\
================================================================================\n\
MSG: roscpp_tutorials/LaneBoundaryType\n\
    uint16 shape\n\
    float32 start_s\n\
    float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/CurveSign\n\
    string id\n\
    uint16 type\n\
    string other_type\n\
    Curved segments\n\
    float32 start_s\n\
    float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/PolygonSign\n\
    string id\n\
    uint16 type\n\
    string other_type\n\
    Curved points\n\
    float32 start_s\n\
    float32 end_s\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/Signal\n\
    string id\n\
    uint16 type\n\
    float32[3] position\n\
    Curved roi_polygon\n\
    Curved stop_line\n\
    float32 start_s\n\
    float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/ParkingSpace\n\
string id\n\
uint16 type\n\
Curved polygon\n\
float32 heading\n\
float32 start_s\n\
float32 end_s\n\
";
  }

  static const char* value(const ::roscpp_tutorials::NavigationLane_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.central_curve);
      stream.next(m.left_samples);
      stream.next(m.right_samples);
      stream.next(m.type);
      stream.next(m.speed_limit);
      stream.next(m.left_boundary);
      stream.next(m.right_boundary);
      stream.next(m.predecessor_id);
      stream.next(m.sucessor_id);
      stream.next(m.left_neighbor_id);
      stream.next(m.right_neighbor_id);
      stream.next(m.curve_signs);
      stream.next(m.polygon_signs);
      stream.next(m.signals);
      stream.next(m.parking_spaces);
      stream.next(m.turn);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavigationLane_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::NavigationLane_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::NavigationLane_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.id);
    s << indent << "central_curve: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::Curved_<ContainerAllocator> >::stream(s, indent + "  ", v.central_curve);
    s << indent << "left_samples[]" << std::endl;
    for (size_t i = 0; i < v.left_samples.size(); ++i)
    {
      s << indent << "  left_samples[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> >::stream(s, indent + "    ", v.left_samples[i]);
    }
    s << indent << "right_samples[]" << std::endl;
    for (size_t i = 0; i < v.right_samples.size(); ++i)
    {
      s << indent << "  right_samples[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> >::stream(s, indent + "    ", v.right_samples[i]);
    }
    s << indent << "type: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.type);
    s << indent << "speed_limit: ";
    Printer<float>::stream(s, indent + "  ", v.speed_limit);
    s << indent << "left_boundary: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >::stream(s, indent + "  ", v.left_boundary);
    s << indent << "right_boundary: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >::stream(s, indent + "  ", v.right_boundary);
    s << indent << "predecessor_id[]" << std::endl;
    for (size_t i = 0; i < v.predecessor_id.size(); ++i)
    {
      s << indent << "  predecessor_id[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.predecessor_id[i]);
    }
    s << indent << "sucessor_id[]" << std::endl;
    for (size_t i = 0; i < v.sucessor_id.size(); ++i)
    {
      s << indent << "  sucessor_id[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.sucessor_id[i]);
    }
    s << indent << "left_neighbor_id[]" << std::endl;
    for (size_t i = 0; i < v.left_neighbor_id.size(); ++i)
    {
      s << indent << "  left_neighbor_id[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.left_neighbor_id[i]);
    }
    s << indent << "right_neighbor_id[]" << std::endl;
    for (size_t i = 0; i < v.right_neighbor_id.size(); ++i)
    {
      s << indent << "  right_neighbor_id[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.right_neighbor_id[i]);
    }
    s << indent << "curve_signs[]" << std::endl;
    for (size_t i = 0; i < v.curve_signs.size(); ++i)
    {
      s << indent << "  curve_signs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >::stream(s, indent + "    ", v.curve_signs[i]);
    }
    s << indent << "polygon_signs[]" << std::endl;
    for (size_t i = 0; i < v.polygon_signs.size(); ++i)
    {
      s << indent << "  polygon_signs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::PolygonSign_<ContainerAllocator> >::stream(s, indent + "    ", v.polygon_signs[i]);
    }
    s << indent << "signals[]" << std::endl;
    for (size_t i = 0; i < v.signals.size(); ++i)
    {
      s << indent << "  signals[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::Signal_<ContainerAllocator> >::stream(s, indent + "    ", v.signals[i]);
    }
    s << indent << "parking_spaces[]" << std::endl;
    for (size_t i = 0; i < v.parking_spaces.size(); ++i)
    {
      s << indent << "  parking_spaces[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >::stream(s, indent + "    ", v.parking_spaces[i]);
    }
    s << indent << "turn: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.turn);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_NAVIGATIONLANE_H
