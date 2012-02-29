/* Auto-generated by genmsg_cpp for file /home/marcello/robotower/SpyKee/msg/Vision.msg */
#ifndef SPYKEE_MESSAGE_VISION_H
#define SPYKEE_MESSAGE_VISION_H
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
struct Vision_ {
  typedef Vision_<ContainerAllocator> Type;

  Vision_()
  : rcvd(false)
  {
  }

  Vision_(const ContainerAllocator& _alloc)
  : rcvd(false)
  {
  }

  typedef uint8_t _rcvd_type;
  uint8_t rcvd;


private:
  static const char* __s_getDataType_() { return "SpyKee/Vision"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "ef16f97691d5a33b2bd3dca95054413d"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "bool rcvd \n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, rcvd);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, rcvd);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(rcvd);
    return size;
  }

  typedef boost::shared_ptr< ::SpyKee::Vision_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::SpyKee::Vision_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Vision
typedef  ::SpyKee::Vision_<std::allocator<void> > Vision;

typedef boost::shared_ptr< ::SpyKee::Vision> VisionPtr;
typedef boost::shared_ptr< ::SpyKee::Vision const> VisionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::SpyKee::Vision_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::SpyKee::Vision_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace SpyKee

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::SpyKee::Vision_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::SpyKee::Vision_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::SpyKee::Vision_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ef16f97691d5a33b2bd3dca95054413d";
  }

  static const char* value(const  ::SpyKee::Vision_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xef16f97691d5a33bULL;
  static const uint64_t static_value2 = 0x2bd3dca95054413dULL;
};

template<class ContainerAllocator>
struct DataType< ::SpyKee::Vision_<ContainerAllocator> > {
  static const char* value() 
  {
    return "SpyKee/Vision";
  }

  static const char* value(const  ::SpyKee::Vision_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::SpyKee::Vision_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool rcvd \n\
\n\
";
  }

  static const char* value(const  ::SpyKee::Vision_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::SpyKee::Vision_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::SpyKee::Vision_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.rcvd);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Vision_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::SpyKee::Vision_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::SpyKee::Vision_<ContainerAllocator> & v) 
  {
    s << indent << "rcvd: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.rcvd);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SPYKEE_MESSAGE_VISION_H

