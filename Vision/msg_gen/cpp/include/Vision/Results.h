/* Auto-generated by genmsg_cpp for file /home/dave/RoboTower/Vision/msg/Results.msg */
#ifndef VISION_MESSAGE_RESULTS_H
#define VISION_MESSAGE_RESULTS_H
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


namespace Vision
{
template <class ContainerAllocator>
struct Results_ {
  typedef Results_<ContainerAllocator> Type;

  Results_()
  : towerFound(false)
  , towerPos(0)
  , towerDistance(0)
  , towerBlobHeight(0)
  , towerBlobWidth(0)
  , factoryFound(false)
  , factoryPos(0)
  , factoryDistance(0)
  , factoryBlobHeight(0)
  , factoryBlobWidth(0)
  {
  }

  Results_(const ContainerAllocator& _alloc)
  : towerFound(false)
  , towerPos(0)
  , towerDistance(0)
  , towerBlobHeight(0)
  , towerBlobWidth(0)
  , factoryFound(false)
  , factoryPos(0)
  , factoryDistance(0)
  , factoryBlobHeight(0)
  , factoryBlobWidth(0)
  {
  }

  typedef uint8_t _towerFound_type;
  uint8_t towerFound;

  typedef int32_t _towerPos_type;
  int32_t towerPos;

  typedef int32_t _towerDistance_type;
  int32_t towerDistance;

  typedef int32_t _towerBlobHeight_type;
  int32_t towerBlobHeight;

  typedef int32_t _towerBlobWidth_type;
  int32_t towerBlobWidth;

  typedef uint8_t _factoryFound_type;
  uint8_t factoryFound;

  typedef int32_t _factoryPos_type;
  int32_t factoryPos;

  typedef int32_t _factoryDistance_type;
  int32_t factoryDistance;

  typedef int32_t _factoryBlobHeight_type;
  int32_t factoryBlobHeight;

  typedef int32_t _factoryBlobWidth_type;
  int32_t factoryBlobWidth;


private:
  static const char* __s_getDataType_() { return "Vision/Results"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "5676373599be0ea9c1c86d1e0c267a3e"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "bool    towerFound\n\
int32    towerPos\n\
int32    towerDistance\n\
int32    towerBlobHeight\n\
int32    towerBlobWidth\n\
bool    factoryFound\n\
int32    factoryPos\n\
int32    factoryDistance\n\
int32    factoryBlobHeight\n\
int32    factoryBlobWidth\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, towerFound);
    ros::serialization::serialize(stream, towerPos);
    ros::serialization::serialize(stream, towerDistance);
    ros::serialization::serialize(stream, towerBlobHeight);
    ros::serialization::serialize(stream, towerBlobWidth);
    ros::serialization::serialize(stream, factoryFound);
    ros::serialization::serialize(stream, factoryPos);
    ros::serialization::serialize(stream, factoryDistance);
    ros::serialization::serialize(stream, factoryBlobHeight);
    ros::serialization::serialize(stream, factoryBlobWidth);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, towerFound);
    ros::serialization::deserialize(stream, towerPos);
    ros::serialization::deserialize(stream, towerDistance);
    ros::serialization::deserialize(stream, towerBlobHeight);
    ros::serialization::deserialize(stream, towerBlobWidth);
    ros::serialization::deserialize(stream, factoryFound);
    ros::serialization::deserialize(stream, factoryPos);
    ros::serialization::deserialize(stream, factoryDistance);
    ros::serialization::deserialize(stream, factoryBlobHeight);
    ros::serialization::deserialize(stream, factoryBlobWidth);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(towerFound);
    size += ros::serialization::serializationLength(towerPos);
    size += ros::serialization::serializationLength(towerDistance);
    size += ros::serialization::serializationLength(towerBlobHeight);
    size += ros::serialization::serializationLength(towerBlobWidth);
    size += ros::serialization::serializationLength(factoryFound);
    size += ros::serialization::serializationLength(factoryPos);
    size += ros::serialization::serializationLength(factoryDistance);
    size += ros::serialization::serializationLength(factoryBlobHeight);
    size += ros::serialization::serializationLength(factoryBlobWidth);
    return size;
  }

  typedef boost::shared_ptr< ::Vision::Results_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Vision::Results_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Results
typedef  ::Vision::Results_<std::allocator<void> > Results;

typedef boost::shared_ptr< ::Vision::Results> ResultsPtr;
typedef boost::shared_ptr< ::Vision::Results const> ResultsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Vision::Results_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Vision::Results_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Vision

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Vision::Results_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Vision::Results_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Vision::Results_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5676373599be0ea9c1c86d1e0c267a3e";
  }

  static const char* value(const  ::Vision::Results_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5676373599be0ea9ULL;
  static const uint64_t static_value2 = 0xc1c86d1e0c267a3eULL;
};

template<class ContainerAllocator>
struct DataType< ::Vision::Results_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Vision/Results";
  }

  static const char* value(const  ::Vision::Results_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Vision::Results_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool    towerFound\n\
int32    towerPos\n\
int32    towerDistance\n\
int32    towerBlobHeight\n\
int32    towerBlobWidth\n\
bool    factoryFound\n\
int32    factoryPos\n\
int32    factoryDistance\n\
int32    factoryBlobHeight\n\
int32    factoryBlobWidth\n\
\n\
";
  }

  static const char* value(const  ::Vision::Results_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::Vision::Results_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Vision::Results_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.towerFound);
    stream.next(m.towerPos);
    stream.next(m.towerDistance);
    stream.next(m.towerBlobHeight);
    stream.next(m.towerBlobWidth);
    stream.next(m.factoryFound);
    stream.next(m.factoryPos);
    stream.next(m.factoryDistance);
    stream.next(m.factoryBlobHeight);
    stream.next(m.factoryBlobWidth);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Results_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Vision::Results_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Vision::Results_<ContainerAllocator> & v) 
  {
    s << indent << "towerFound: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.towerFound);
    s << indent << "towerPos: ";
    Printer<int32_t>::stream(s, indent + "  ", v.towerPos);
    s << indent << "towerDistance: ";
    Printer<int32_t>::stream(s, indent + "  ", v.towerDistance);
    s << indent << "towerBlobHeight: ";
    Printer<int32_t>::stream(s, indent + "  ", v.towerBlobHeight);
    s << indent << "towerBlobWidth: ";
    Printer<int32_t>::stream(s, indent + "  ", v.towerBlobWidth);
    s << indent << "factoryFound: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.factoryFound);
    s << indent << "factoryPos: ";
    Printer<int32_t>::stream(s, indent + "  ", v.factoryPos);
    s << indent << "factoryDistance: ";
    Printer<int32_t>::stream(s, indent + "  ", v.factoryDistance);
    s << indent << "factoryBlobHeight: ";
    Printer<int32_t>::stream(s, indent + "  ", v.factoryBlobHeight);
    s << indent << "factoryBlobWidth: ";
    Printer<int32_t>::stream(s, indent + "  ", v.factoryBlobWidth);
  }
};


} // namespace message_operations
} // namespace ros

#endif // VISION_MESSAGE_RESULTS_H

