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

#ifndef MYPROTOCOL_PACKET_H
#define MYPROTOCOL_PACKET_H

#include <iostream>
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"

namespace ns3 {
namespace myprotocol {
/**
 * \ingroup myprotocol
 * \brief myprotocol Update Packet Format
 * \verbatim
 |      0        |      1        |      2        |       3       |
  0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                      Destination Address                      |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                            HopCount                           |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                       Sequence Number                         |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * \endverbatim
 */

class MyprotocolHeader : public Header
{
public:
  /**
   * Constructor
   *
   * \param dst destination IP address
   * \param hopcount hop count
   * \param dstSeqNo destination sequence number
   */
  MyprotocolHeader (Ipv4Address dst = Ipv4Address (), uint32_t hopcount = 0, uint32_t dstSeqNo = 0,
                    uint16_t x = 0, uint16_t y = 0, uint16_t z = 0, uint16_t vx = 0, uint16_t vy = 0, uint16_t vz = 0, uint16_t sign = 0, 
                    uint16_t timestamp = 0, Ipv4Address myadress = Ipv4Address ());
  virtual ~MyprotocolHeader ();
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize () const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;

  /**
   * Set destination address
   * \param destination the destination IPv4 address
   */
  void
  SetDst (Ipv4Address destination)
  {
    m_dst = destination;
  }
  /**
   * Get destination address
   * \returns the destination IPv4 address
   */
  Ipv4Address
  GetDst () const
  {
    return m_dst;
  }
  /**
   * Set hop count
   * \param hopCount the hop count
   */
  void
  SetHopCount (uint32_t hopCount)
  {
    m_hopCount = hopCount;
  }
  /**
   * Get hop count
   * \returns the hop count
   */
  uint32_t
  GetHopCount () const
  {
    return m_hopCount;
  }
  /**
   * Set destination sequence number
   * \param sequenceNumber The sequence number
   */
  void
  SetDstSeqno (uint32_t sequenceNumber)
  {
    m_dstSeqNo = sequenceNumber;
  }
  /**
   * Get destination sequence number
   * \returns the destination sequence number
   */
  uint32_t
  GetDstSeqno () const
  {
    return m_dstSeqNo;
  }

  //ADD
  void SetX(uint16_t x){
    m_x = x;
  }
  uint16_t GetX() const{
    return m_x;
  }
  void SetY(uint16_t y){
    m_y = y;
  }
  uint16_t GetY() const{
    return m_y;
  }
  void SetZ(uint16_t z){
    m_z = z;
  }
  uint16_t GetZ() const{
    return m_z;
  }
  void SetVx(uint16_t vx){
    m_vx = vx;
  }
  uint16_t GetVx() const{
    return m_vx;
  }
  void SetVy(uint16_t vy){
    m_vy = vy;
  }
  uint16_t GetVy() const{
    return m_vy;
  }
  void SetVz(uint16_t vz){
    m_vz = vz;
  }
  uint16_t GetVz() const{
    return m_vz;
  }
  void SetSign(uint16_t sign){
    m_sign = sign;
  }
  uint16_t GetSign() const{
    return m_sign;
  }
  void SetTimestamp(uint16_t timestamp){
    m_timestamp = timestamp;
  }
  uint16_t GetTimestamp() const{
    return m_timestamp;
  }
  void
  SetMyadress (Ipv4Address myadress)
  {
    m_myadress = myadress;
  }
  Ipv4Address
  GetMyadress () const
  {
    return m_myadress;
  }
private:
  Ipv4Address m_dst; ///< Destination IP Address
  uint32_t m_hopCount; ///< Number of Hops
  uint32_t m_dstSeqNo; ///< Destination Sequence Number
  //ADD:添加位置信息、速度信息、时间戳
  uint16_t m_x;
  uint16_t m_y;
  uint16_t m_z;
  uint16_t m_vx;
  uint16_t m_vy;
  uint16_t m_vz;
  uint16_t m_sign;      //记录速度是否为负数，0:都不是负数，1:X轴速度为负，2:Y轴速度为负，3:Z轴速度为负,4：xy为负数，5：xz为负数，6：yz为负数，7：全部都是负数
  //以秒为单位的整数，每次重新运行代码都是从0s开始
  uint16_t m_timestamp;
  Ipv4Address m_myadress;
};
static inline std::ostream & operator<< (std::ostream& os, const MyprotocolHeader & packet)
{
  packet.Print (os);
  return os;
}
}
}

#endif /* MYPROTOCOL_PACKET_H */
