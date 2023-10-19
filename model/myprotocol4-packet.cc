#include "myprotocol4-packet.h"
#include "ns3/address-utils.h"
#include "ns3/packet.h"

namespace ns3 {
namespace myprotocol4 {

NS_OBJECT_ENSURE_REGISTERED (MyprotocolHeader);

MyprotocolHeader::MyprotocolHeader (uint16_t x, uint16_t y, uint16_t z, uint16_t vx, uint16_t vy, uint16_t vz, uint16_t sign, 
                                    uint16_t timestamp, Ipv4Address myadress, uint64_t uid)
  : m_x(x),
    m_y(y),
    m_z(z),
    m_vx(vx),
    m_vy(vy),
    m_vz(vz),
    m_sign(sign),
    m_timestamp(timestamp),
    m_myadress(myadress),
    m_uid(uid)
{
}

MyprotocolHeader::~MyprotocolHeader ()
{
}

TypeId
MyprotocolHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::myprotocol4::MyprotocolHeader")
    .SetParent<Header> ()
    .SetGroupName ("Myprotocol4")
    .AddConstructor<MyprotocolHeader> ();
  return tid;
}

TypeId
MyprotocolHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

// 包头长度：8*1 + 4*1 + 2*8 = 28
uint32_t
MyprotocolHeader::GetSerializedSize () const
{
  return 28;
}

void
MyprotocolHeader::Serialize (Buffer::Iterator i) const
{
  // ADD: 序列化位置信息
  i.WriteHtonU16 (m_x);
  i.WriteHtonU16 (m_y);
  i.WriteHtonU16 (m_z);
  i.WriteHtonU16 (m_vx);
  i.WriteHtonU16 (m_vy);
  i.WriteHtonU16 (m_vz);
  i.WriteHtonU16(m_sign);
  i.WriteHtonU16 (m_timestamp);
  i.WriteHtonU64 (m_uid);
  WriteTo (i, m_myadress);
}

uint32_t
MyprotocolHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  //ADD: 反序列化位置信息
  m_x = i.ReadNtohU16 ();
  m_y = i.ReadNtohU16 ();
  m_z = i.ReadNtohU16 ();
  m_vx = i.ReadNtohU16 ();
  m_vy = i.ReadNtohU16 ();
  m_vz = i.ReadNtohU16 ();
  m_sign = i.ReadNtohU16();
  m_timestamp = i.ReadNtohU16 ();
  m_uid = i.ReadNtohU64 ();
  ReadFrom (i, m_myadress);

  uint32_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize ());
  return dist;
}

void
MyprotocolHeader::Print (std::ostream &os) const
{
  os << " length: " << GetSerializedSize()
     << " X: " << m_x
     << " Y: " << m_y
     << " Z: " << m_z
     << " VX: " << m_vx
     << " VY: " << m_vy
     << " VZ: " << m_vz
     << " sign: "<<m_sign
     << " timestamp: "<<m_timestamp
     << " myadress: "<<m_myadress
     << " uid: "<<m_uid;
}

NS_OBJECT_ENSURE_REGISTERED (DataHeader);

DataHeader::DataHeader (uint16_t dstPosx, uint16_t dstPosy, uint16_t dstPosz, 
                        uint16_t dstVelx, uint16_t dstVely, uint16_t dstVelz, 
                        uint16_t dstSign, uint16_t dstTimestamp,
                        uint16_t recPosx, uint16_t recPosy, uint16_t recPosz, uint16_t inRec,
                        uint64_t uid, uint16_t hop)
  : m_dstPosx(dstPosx),
    m_dstPosy(dstPosy),
    m_dstPosz(dstPosz),
    m_dstVelx(dstVelx),
    m_dstVely(dstVely),
    m_dstVelz(dstVelz),
    m_dstSign(dstSign),
    m_dstTimestamp(dstTimestamp),
    m_recPosx (recPosx),
    m_recPosy (recPosy),
    m_recPosz (recPosz),
    m_inRec (inRec),
    m_uid(uid),
    m_hop(hop)
{
}

TypeId
DataHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::myprotocol4::DataHeader")
    .SetParent<Header> ()
    .AddConstructor<DataHeader> ()
  ;
  return tid;
}

TypeId
DataHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

// 数据头大小2*13 + 8*1 = 34
uint32_t
DataHeader::GetSerializedSize () const
{
  return 34;
}

void
DataHeader::Serialize (Buffer::Iterator i) const
{
  i.WriteHtonU16 (m_dstPosx);
  i.WriteHtonU16 (m_dstPosy);
  i.WriteHtonU16 (m_dstPosz);
  i.WriteHtonU16 (m_dstVelx);
  i.WriteHtonU16 (m_dstVely);
  i.WriteHtonU16 (m_dstVelz);
  i.WriteHtonU16 (m_dstSign);
  i.WriteHtonU16 (m_dstTimestamp);
  i.WriteHtonU16 (m_recPosx);
  i.WriteHtonU16 (m_recPosy);
  i.WriteHtonU16 (m_recPosz);
  i.WriteHtonU16 (m_inRec);
  i.WriteHtonU64 (m_uid);
  i.WriteHtonU16 (m_hop);
}

uint32_t
DataHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  m_dstPosx = i.ReadNtohU16 ();
  m_dstPosy = i.ReadNtohU16 ();
  m_dstPosz = i.ReadNtohU16 ();
  m_dstVelx = i.ReadNtohU16 ();
  m_dstVely = i.ReadNtohU16 ();
  m_dstVelz = i.ReadNtohU16 ();
  m_dstSign = i.ReadNtohU16 ();
  m_dstTimestamp = i.ReadNtohU16 ();
  m_recPosx = i.ReadNtohU16 ();
  m_recPosy = i.ReadNtohU16 ();
  m_recPosz = i.ReadNtohU16 ();
  m_inRec = i.ReadNtohU16 ();
  m_uid = i.ReadNtohU64 ();
  m_hop = i.ReadNtohU16 ();

  uint32_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize ());
  return dist;
}

void
DataHeader::Print (std::ostream &os) const
{
  os << " length: " << GetSerializedSize()
     << " dstPositionX: "  << m_dstPosx
     << " dstPositionY: " << m_dstPosy
     << " dstPositionZ: " << m_dstPosx
     << " dstVelocityX: " << m_dstVelx
     << " dstVelocityY: " << m_dstVely
     << " dstVelocityZ: " << m_dstVelz
     << " dstSign: " << m_dstSign
     << " dstTimestamp: " << m_dstTimestamp
     << " RecPositionX: " << m_recPosx
     << " RecPositionY: " << m_recPosy
     << " RecPositionZ: " << m_recPosz
     << " inRec: " << m_inRec
     << " uid: "<<m_uid
     << " hop: "<<m_hop;
}

std::ostream &
operator<< (std::ostream & os, DataHeader const & h)
{
  h.Print (os);
  return os;
}

bool
DataHeader::operator== (DataHeader const & o) const
{
  return (m_dstPosx == o.m_dstPosx && m_dstPosy == o.m_dstPosy && m_dstPosz == o.m_dstPosz &&
          m_dstVelx == o.m_dstVelx && m_dstVely == o.m_dstVely && m_dstVelz == o.m_dstVelz &&
          m_dstSign == o.m_dstSign && m_dstTimestamp == o.m_dstTimestamp &&
          m_recPosx == o.m_recPosx && m_recPosy == o.m_recPosy && m_recPosz == o.m_recPosz &&
           m_inRec == o.m_inRec && m_uid == o.m_uid && m_hop == o.m_hop);
}
}
}
