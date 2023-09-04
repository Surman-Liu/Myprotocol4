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

#ifndef MYPROTOCOL_RTABLE_H
#define MYPROTOCOL_RTABLE_H

#include <cassert>
#include <map>
#include <sys/types.h>
#include "ns3/ipv4.h"
#include "ns3/ipv4-route.h"
#include "ns3/timer.h"
#include "ns3/net-device.h"
#include "ns3/output-stream-wrapper.h"
// ADD：添加Vector
#include "ns3/vector.h"
// 添加移动模型
#include "ns3/mobility-model.h"

namespace ns3 {
namespace myprotocol {

/**
 * \ingroup myprotocol
 * \brief Routing table entry
 */
class RoutingTableEntry
{
public:
  /**
   * c-tor
   *
   * \param dev the net device
   * \param dst the destination IP address
   * \param seqNo the sequence number
   * \param iface the interface
   * \param hops the number of hops
   * \param nextHop the IP address of the next hop
   * \param lifetime the lifetime 
   * \param SettlingTime the settling time
   * \param changedEntries flag for changed entries
   */
  // ADD:在构造函数中添加位置、速度、时间戳
  RoutingTableEntry (uint16_t x = 0,uint16_t y = 0,uint16_t z = 0, 
                    int16_t vx = 0,int16_t vy = 0,int16_t vz = 0,
                    uint16_t timestamp = 0, Ipv4Address adress = Ipv4Address ());

  ~RoutingTableEntry ();

  /**
   * Print routing table entry
   * \param stream the output stream
   */
  void
  Print (Ptr<OutputStreamWrapper> stream) const;

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
  void SetVx(int16_t vx){
    m_vx = vx;
  }
  int16_t GetVx() const{
    return m_vx;
  }
  void SetVy(int16_t vy){
    m_vy = vy;
  }
  int16_t GetVy() const{
    return m_vy;
  }
  void SetVz(int16_t vz){
    m_vz = vz;
  }
  int16_t GetVz() const{
    return m_vz;
  }
  void SetTimestamp(uint16_t timestamp){
    m_timestamp = timestamp;
  }
  uint16_t GetTimestamp() const{
    return m_timestamp;
  }
  void SetAdress (Ipv4Address adress)
  {
    m_adress = adress;
  }
  Ipv4Address GetAdress () const
  {
    return m_adress;
  }

private:
  //ADD: 当前位置、速度、时间戳
  uint16_t m_x;
  uint16_t m_y;
  uint16_t m_z;
  int16_t m_vx;
  int16_t m_vy;
  int16_t m_vz;
  uint16_t m_timestamp;
  Ipv4Address m_adress;
};

/**
 * \ingroup myprotocol
 * \brief The Routing table used by myprotocol protocol
 */
class RoutingTable
{
public:
  /// c-tor
  RoutingTable ();
  /**
   * Add routing table entry if it doesn't yet exist in routing table
   * \param r routing table entry
   * \return true in success
   */
  bool
  AddRoute (RoutingTableEntry & r);
  /**
   * Delete routing table entry with destination address dst, if it exists.
   * \param dst destination address
   * \return true on success
   */
  bool
  DeleteRoute (Ipv4Address dst);
  /**
   * Lookup routing table entry with destination address dst
   * \param dst destination address
   * \param rt entry with destination address dst, if exists
   * \return true on success
   */
  bool
  LookupRoute (Ipv4Address dst, RoutingTableEntry & rt);
  /**
   * Updating the routing Table with routing table entry rt
   * \param rt routing table entry
   * \return true on success
   */
  bool
  Update (RoutingTableEntry & rt);
  /// Delete all entries from routing table
  void
  Clear ()
  {
    m_positionTable.clear ();
  }
  /**
   * Print routing table
   * \param stream the output stream
   */
  void
  Print (Ptr<OutputStreamWrapper> stream) const;

  // ADD: 位置预测函数
  Vector PredictPosition(Ipv4Address id); 

  // ADD：筛选邻居节点
  void LookupNeighbor(std::map<Ipv4Address, RoutingTableEntry> & neighborTable, Vector myPos);

  // ADD:贪婪寻找最优下一跳的函数
  /**
   * \brief Gets next hop according to GPSR protocol
   * \param position the position of the destination node
   * \param nodePos the position of the node that has the packet
   * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
   */
  Ipv4Address BestNeighbor (std::map<Ipv4Address, RoutingTableEntry> neighborTable, Vector dstPos, Vector myPos);    //dstPos需要时经过预测后的目的地位置

  void Purge();

private:
  // Fields
  // 表项过期时间
  uint16_t m_entryLifeTime;
  /// an entry in the routing table.
  std::map<Ipv4Address, RoutingTableEntry> m_positionTable;
  /// neighbor table
  std::map<Ipv4Address, RoutingTableEntry> m_neiborTable;
};
}
}
#endif /* MYPROTOCOL_RTABLE_H */
