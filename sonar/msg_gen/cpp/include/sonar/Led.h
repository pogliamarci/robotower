/* Auto-generated by genmsg_cpp for file /home/dave/RoboTower/sonar/msg/Led.msg */
#ifndef SONAR_MESSAGE_LED_H
#define SONAR_MESSAGE_LED_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace sonar
{
template <class ContainerAllocator>
struct Led_ {
  typedef Led_<ContainerAllocator> Type;

  Led_()
  : greenOn(false)
  , numRedOn(0)
  , yellowOn()
  {
    yellowOn.assign(false);
  }

  Led_(const ContainerAllocator& _alloc)
  : greenOn(false)
  , numRedOn(0)
  , yellowOn()
  {
    yellowOn.assign(false);
  }

  typedef uint8_t _greenOn_type;
  uint8_t greenOn;

  typedef uint8_t _numRedOn_type;
  uint8_t numRedOn;

  typedef boost::array<uint8_t, 4>  _yellowOn_type;
  boost::array<uint8_t, 4>  yellowOn;


  ROS_DEPRECATED uint32_t get_yellowOn_size() const { return (uint32_t)yellowOn.size(); }
private:
  static const char* __s_getDataType_() { return "sonar/Led"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "b4f59e8c864d23e3c9d2105fe9c83902"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "bool greenOn\n\
uint8 numRedOn\n\
bool[4] yellowOn\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, greenOn);
    ros::serialization::serialize(stream, numRedOn);
    ros::serialization::serialize(stream, yellowOn);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, greenOn);
    ros::serialization::deserialize(stream, numRedOn);
    ros::serialization::deserialize(stream, yellowOn);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(greenOn);
    size += ros::serialization::serializationLength(numRedOn);
    size += ros::serialization::serializationLength(yellowOn);
    return size;
  }

  typedef boost::shared_ptr< ::sonar::Led_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sonar::Led_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Led
typedef  ::sonar::Led_<std::allocator<void> > Led;

typedef boost::shared_ptr< ::sonar::Led> LedPtr;
typedef boost::shared_ptr< ::sonar::Led const> LedConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::sonar::Led_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::sonar::Led_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace sonar

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::sonar::Led_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::sonar::Led_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::sonar::Led_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b4f59e8c864d23e3c9d2105fe9c83902";
  }

  static const char* value(const  ::sonar::Led_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb4f59e8c864d23e3ULL;
  static const uint64_t static_value2 = 0xc9d2105fe9c83902ULL;
};

template<class ContainerAllocator>
struct DataType< ::sonar::Led_<ContainerAllocator> > {
  static const char* value() 
  {
    return "sonar/Led";
  }

  static const char* value(const  ::sonar::Led_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::sonar::Led_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool greenOn\n\
uint8 numRedOn\n\
bool[4] yellowOn\n\
\n\
";
  }

  static const char* value(const  ::sonar::Led_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::sonar::Led_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::sonar::Led_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.greenOn);
    stream.next(m.numRedOn);
    stream.next(m.yellowOn);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Led_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sonar::Led_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::sonar::Led_<ContainerAllocator> & v) 
  {
    s << indent << "greenOn: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.greenOn);
    s << indent << "numRedOn: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.numRedOn);
    s << indent << "yellowOn[]" << std::endl;
    for (size_t i = 0; i < v.yellowOn.size(); ++i)
    {
      s << indent << "  yellowOn[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.yellowOn[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // SONAR_MESSAGE_LED_H

