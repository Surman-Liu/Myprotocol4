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

MyprotocolHeader::MyprotocolHeader (Ipv4Address dst, uint32_t hopCount, uint32_t dstSeqNo,
                                    uint16_t x, uint16_t y, uint16_t z, uint16_t vx, uint16_t vy, uint16_t vz, uint16_t sign, 
                                    uint16_t timestamp, Ipv4Address myadress)
  : m_dst (dst),
    m_hopCount (hopCount),
    m_dstSeqNo (dstSeqNo),
    m_x(x),
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

// 包头长度：4*4 + 2*8 = 32
uint32_t
MyprotocolHeader::GetSerializedSize () const
{
  return 32;
}

void
MyprotocolHeader::Serialize (Buffer::Iterator i) const
{
  WriteTo (i, m_dst);
  i.WriteU32(m_hopCount);
  i.WriteU32 (m_dstSeqNo);
  // ADD: 序列化位置信息
  i.WriteU16 (m_x);
  i.WriteU16 (m_y);
  i.WriteU16 (m_z);
  i.WriteU16 (m_vx);
  i.WriteU16 (m_vy);
  i.WriteU16 (m_vz);
  i.WriteU16(m_sign);
  i.WriteU16 (m_timestamp);
  WriteTo (i, m_myadress);
}

uint32_t
MyprotocolHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  ReadFrom (i, m_dst);
  m_hopCount = i.ReadU32 ();
  m_dstSeqNo = i.ReadU32 ();
  //ADD: 反序列化位置信息
  m_x = i.ReadU16 ();
  m_y = i.ReadU16 ();
  m_z = i.ReadU16 ();
  m_vx = i.ReadU16 ();
  m_vy = i.ReadU16 ();
  m_vz = i.ReadU16 ();
  m_sign = i.ReadU16();
  m_timestamp = i.ReadU16 ();
  ReadFrom (i, m_myadress);

  uint32_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize ());
  return dist;
}

void
MyprotocolHeader::Print (std::ostream &os) const
{
  os << "DestinationIpv4: " << m_dst
     << " Hopcount: " << m_hopCount
     << " SequenceNumber: " << m_dstSeqNo
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
}
}
