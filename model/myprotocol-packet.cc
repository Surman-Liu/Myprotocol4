/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 Hemanth Narra
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Hemanth Narra <hemanth@ittc.ku.com>
 *
 * James P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  http://wiki.ittc.ku.edu/resilinets
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 *
 * Work supported in part by NSF FIND (Future Internet Design) Program
 * under grant CNS-0626918 (Postmodern Internet Architecture),
 * NSF grant CNS-1050226 (Multilayer Network Resilience Analysis and Experimentation on GENI),
 * US Department of Defense (DoD), and ITTC at The University of Kansas.
 */
#include "myprotocol-packet.h"
#include "ns3/address-utils.h"
#include "ns3/packet.h"

namespace ns3 {
namespace myprotocol {

NS_OBJECT_ENSURE_REGISTERED (MyprotocolHeader);

MyprotocolHeader::MyprotocolHeader (uint16_t x, uint16_t y, uint16_t z, uint16_t vx, uint16_t vy, uint16_t vz, uint16_t sign, 
                                    uint16_t timestamp, Ipv4Address myadress)
  : m_x(x),
    m_y(y),
    m_z(z),
    m_vx(vx),
    m_vy(vy),
    m_vz(vz),
    m_sign(sign),
    m_timestamp(timestamp),
    m_myadress(myadress)
{
}

MyprotocolHeader::~MyprotocolHeader ()
{
}

TypeId
MyprotocolHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::myprotocol::MyprotocolHeader")
    .SetParent<Header> ()
    .SetGroupName ("Myprotocol")
    .AddConstructor<MyprotocolHeader> ();
  return tid;
}

TypeId
MyprotocolHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

// 包头长度：4*1 + 2*8 = 20
uint32_t
MyprotocolHeader::GetSerializedSize () const
{
  return 20;
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
     << " myadress: "<<m_myadress;
}

//-----------------------------------------------------------------------------
// Data
//-----------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED (DataHeader);

DataHeader::DataHeader (uint16_t dstPosx, uint16_t dstPosy, uint16_t dstPosz, 
                        uint16_t dstVelx, uint16_t dstVely, uint16_t dstVelz, 
                        uint16_t sign, uint16_t timestamp, 
                        uint16_t recPosx, uint16_t recPosy, uint16_t recPosz, uint16_t inRec)
  : m_dstPosx(dstPosx),
    m_dstPosy(dstPosy),
    m_dstPosz(dstPosz),
    m_dstVelx(dstVelx),
    m_dstVely(dstVely),
    m_dstVelz(dstVelz),
    m_dstSign(sign),
    m_timestamp(timestamp),
    m_recPosx (recPosx),
    m_recPosy (recPosy),
    m_recPosz (recPosz),
    m_inRec (inRec)
{
}

TypeId
DataHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::myprotocol::DataHeader")
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

// 数据头大小2*12 = 24
uint32_t
DataHeader::GetSerializedSize () const
{
  return 24;
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
  i.WriteHtonU16 (m_timestamp);
  i.WriteHtonU16 (m_recPosx);
  i.WriteHtonU16 (m_recPosy);
  i.WriteHtonU16 (m_recPosz);
  i.WriteHtonU16 (m_inRec);
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
  m_timestamp = i.ReadNtohU16 ();
  m_recPosx = i.ReadNtohU16 ();
  m_recPosy = i.ReadNtohU16 ();
  m_recPosz = i.ReadNtohU16 ();
  m_inRec = i.ReadNtohU16 ();

  uint32_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize ());
  return dist;
}

void
DataHeader::Print (std::ostream &os) const
{
  os << " length: " << GetSerializedSize()
     << " PositionX: "  << m_dstPosx
     << " PositionY: " << m_dstPosy
     << " PositionZ: " << m_dstPosx
     << " VelocityX: " << m_dstVelx
     << " VelocityY: " << m_dstVely
     << " VelocityZ: " << m_dstVelz
     << " Sign: " << m_dstSign
     << " Timestamp: " << m_timestamp
     << " RecPositionX: " << m_recPosx
     << " RecPositionY: " << m_recPosy
     << " RecPositionZ: " << m_recPosz
     << " inRec: " << m_inRec;
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
          m_dstSign == o.m_dstSign && m_timestamp == o.m_timestamp &&
          m_recPosx == o.m_recPosx && m_recPosy == o.m_recPosy && m_recPosz == o.m_recPosz && m_inRec == o.m_inRec);
}
}
}
