/* Auto-generated by genmsg_cpp for file /home/marcello/robotower/SpyKee/msg/Motion.msg */
#ifndef SPYKEE_MESSAGE_MOTION_H
#define SPYKEE_MESSAGE_MOTION_H
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


namespace SpyKee
{
template <class ContainerAllocator>
struct Motion_ {
  typedef Motion_<ContainerAllocator> Type;

  Motion_()
  : tanSpeed(0)
  , rotSpeed(0)
  {
  }

  Motion_(const ContainerAllocator& _alloc)
  : tanSpeed(0)
  , rotSpeed(0)
  {
  }

  typedef int8_t _tanSpeed_type;
  int8_t tanSpeed;

  typedef int8_t _rotSpeed_type;
  int8_t rotSpeed;


private:
  static const char* __s_getDataType_() { return "SpyKee/Motion"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "1652fa15339d862f9e744f6e325e4ee4"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int8 tanSpeed\n\
int8 rotSpeed\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, tanSpeed);
    ros::serialization::serialize(stream, rotSpeed);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, tanSpeed);
    ros::serialization::deserialize(stream, rotSpeed);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(tanSpeed);
    size += ros::serialization::serializationLength(rotSpeed);
    return size;
  }

  typedef boost::shared_ptr< ::SpyKee::Motion_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::SpyKee::Motion_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Motion
typedef  ::SpyKee::Motion_<std::allocator<void> > Motion;

typedef boost::shared_ptr< ::SpyKee::Motion> MotionPtr;
typedef boost::shared_ptr< ::SpyKee::Motion const> MotionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::SpyKee::Motion_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::SpyKee::Motion_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace SpyKee

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::SpyKee::Motion_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::SpyKee::Motion_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::SpyKee::Motion_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1652fa15339d862f9e744f6e325e4ee4";
  }

  static const char* value(const  ::SpyKee::Motion_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1652fa15339d862fULL;
  static const uint64_t static_value2 = 0x9e744f6e325e4ee4ULL;
};

template<class ContainerAllocator>
struct DataType< ::SpyKee::Motion_<ContainerAllocator> > {
  static const char* value() 
  {
    return "SpyKee/Motion";
  }

  static const char* value(const  ::SpyKee::Motion_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::SpyKee::Motion_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 tanSpeed\n\
int8 rotSpeed\n\
\n\
";
  }

  static const char* value(const  ::SpyKee::Motion_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::SpyKee::Motion_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::SpyKee::Motion_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.tanSpeed);
    stream.next(m.rotSpeed);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Motion_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::SpyKee::Motion_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::SpyKee::Motion_<ContainerAllocator> & v) 
  {
    s << indent << "tanSpeed: ";
    Printer<int8_t>::stream(s, indent + "  ", v.tanSpeed);
    s << indent << "rotSpeed: ";
    Printer<int8_t>::stream(s, indent + "  ", v.rotSpeed);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SPYKEE_MESSAGE_MOTION_H

