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
enum RouteFlags
{
  VALID = 0,     // !< VALID
  INVALID = 1,     // !< INVALID
};

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
  RoutingTableEntry (Ptr<NetDevice> dev = 0, Ipv4Address dst = Ipv4Address (), uint32_t seqNo = 0,
                     Ipv4InterfaceAddress iface = Ipv4InterfaceAddress (), uint32_t hops = 0, Ipv4Address nextHop = Ipv4Address (),
                     Time lifetime = Simulator::Now (), Time SettlingTime = Simulator::Now (), bool changedEntries = false,
                     uint16_t x = 0,uint16_t y = 0,uint16_t z = 0, int16_t vx = 0,int16_t vy = 0,int16_t vz = 0,
                    uint16_t timestamp = 0, Ipv4Address adress = Ipv4Address ());

  ~RoutingTableEntry ();
  /**
   * Get destination IP address
   * \returns the destination IPv4 address
   */
  Ipv4Address
  GetDestination () const
  {
    return m_ipv4Route->GetDestination ();
  }
  /**
   * Get route
   * \returns the IPv4 route
   */
  Ptr<Ipv4Route>
  GetRoute () const
  {
    return m_ipv4Route;
  }
  /**
   * Set route
   * \param route the IPv4 route
   */
  void
  SetRoute (Ptr<Ipv4Route> route)
  {
    m_ipv4Route = route;
  }
  /**
   * Set next hop
   * \param nextHop the IPv4 address of the next hop
   */
  void
  SetNextHop (Ipv4Address nextHop)
  {
    m_ipv4Route->SetGateway (nextHop);
  }
  /**
   * Get next hop
   * \returns the IPv4 address of the next hop
   */
  Ipv4Address
  GetNextHop () const
  {
    return m_ipv4Route->GetGateway ();
  }
  /**
   * Set output device
   * \param device the output device
   */
  void
  SetOutputDevice (Ptr<NetDevice> device)
  {
    m_ipv4Route->SetOutputDevice (device);
  }
  /**
   * Get output device
   * \returns the output device
   */
  Ptr<NetDevice>
  GetOutputDevice () const
  {
    return m_ipv4Route->GetOutputDevice ();
  }
  /**
   * Get interface address
   * \returns the IPv4 interface address
   */
  Ipv4InterfaceAddress
  GetInterface () const
  {
    return m_iface;
  }
  /**
   * Set interface address
   * \param iface the IPv4 interface address
   */
  void
  SetInterface (Ipv4InterfaceAddress iface)
  {
    m_iface = iface;
  }
  /**
   * Set sequence number
   * \param sequenceNumber the sequence number
   */
  void
  SetSeqNo (uint32_t sequenceNumber)
  {
    m_seqNo = sequenceNumber;
  }
  /**
   * Get sequence number
   * \returns the sequence number
   */
  uint32_t
  GetSeqNo () const
  {
    return m_seqNo;
  }
  /**
   * Set hop
   * \param hopCount the hop count
   */
  void
  SetHop (uint32_t hopCount)
  {
    m_hops = hopCount;
  }
  /**
   * Get hop
   * \returns the hop count
   */
  uint32_t
  GetHop () const
  {
    return m_hops;
  }
  /**
   * Set lifetime
   * \param lifeTime the lifetime value
   */
  void
  SetLifeTime (Time lifeTime)
  {
    m_lifeTime = lifeTime;
  }
  /**
   * Get lifetime
   * \returns the lifetime value
   */
  Time
  GetLifeTime () const
  {
    return (Simulator::Now () - m_lifeTime);
  }
  /**
   * Set settling time
   * \param settlingTime the settling time
   */
  void
  SetSettlingTime (Time settlingTime)
  {
    m_settlingTime = settlingTime;
  }
  /**
   * Get settling time
   * \returns the settling time
   */
  Time
  GetSettlingTime () const
  {
    return (m_settlingTime);
  }
  /**
   * Set route flags
   * \param flag the route flags
   */
  void
  SetFlag (RouteFlags flag)
  {
    m_flag = flag;
  }
  /**
   * Get route flags
   * \returns the route flags
   */
  RouteFlags
  GetFlag () const
  {
    return m_flag;
  }
  /**
   * Set entries changed indicator
   * \param entriesChanged
   */
  void
  SetEntriesChanged (bool entriesChanged)
  {
    m_entriesChanged = entriesChanged;
  }
  /**
   * Get entries changed
   * \returns the entries changed flag
   */
  bool
  GetEntriesChanged () const
  {
    return m_entriesChanged;
  }
  /**
   * \brief Compare destination address
   * \param destination destination node IP address
   * \return true if equal
   */
  bool
  operator== (Ipv4Address const destination) const
  {
    return (m_ipv4Route->GetDestination () == destination);
  }
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
  // Fields
  /// Destination Sequence Number
  uint32_t m_seqNo;
  /// Hop Count (number of hops needed to reach destination)
  uint32_t m_hops;
  /**
   * \brief Expiration or deletion time of the route
   *	Lifetime field in the routing table plays dual role --
   *	for an active route it is the expiration time, and for an invalid route
   *	it is the deletion time.
   */
  Time m_lifeTime;
  /** Ip route, include
   *   - destination address
   *   - source address
   *   - next hop address (gateway)
   *   - output device
   */
  Ptr<Ipv4Route> m_ipv4Route;
  /// Output interface address
  Ipv4InterfaceAddress m_iface;
  /// Routing flags: valid, invalid or in search
  RouteFlags m_flag;
  /// Time for which the node retains an update with changed metric before broadcasting it.
  /// A node does that in hope of receiving a better update.
  Time m_settlingTime;
  /// Flag to show if any of the routing table entries were changed with the routing update.
  uint32_t m_entriesChanged;

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
   * Lookup routing table entry with destination address dst
   * \param id destination address
   * \param rt entry with destination address dst, if exists
   * \param forRouteInput for routing input
   * \return true on success
   */
  bool
  LookupRoute (Ipv4Address id, RoutingTableEntry & rt, bool forRouteInput);
  /**
   * Updating the routing Table with routing table entry rt
   * \param rt routing table entry
   * \return true on success
   */
  bool
  Update (RoutingTableEntry & rt);
  /**
   * Lookup list of addresses for which nxtHp is the next Hop address
   * \param nxtHp nexthop's address for which we want the list of destinations
   * \param dstList is the list that will hold all these destination addresses
   */
  void
  GetListOfDestinationWithNextHop (Ipv4Address nxtHp, std::map<Ipv4Address, RoutingTableEntry> & dstList);
  /**
   * Lookup list of all addresses in the routing table
   * \param allRoutes is the list that will hold all these addresses present in the nodes routing table
   */
  void
  GetListOfAllRoutes (std::map<Ipv4Address, RoutingTableEntry> & allRoutes);
  /**
   * Delete all route from interface with address iface
   * \param iface the interface
   */
  void
  DeleteAllRoutesFromInterface (Ipv4InterfaceAddress iface);
  /// Delete all entries from routing table
  void
  Clear ()
  {
    m_ipv4AddressEntry.clear ();
  }
  /**
   * Delete all outdated entries if Lifetime is expired
   * \param removedAddresses is the list of addresses to purge
   */
  void
  Purge (std::map<Ipv4Address, RoutingTableEntry> & removedAddresses);
  /**
   * Print routing table
   * \param stream the output stream
   */
  void
  Print (Ptr<OutputStreamWrapper> stream) const;
  /**
   * Provides the number of routes present in that nodes routing table.
   * \returns the number of routes
   */
  uint32_t
  RoutingTableSize ();
  /**
  * Add an event for a destination address so that the update to for that destination is sent
  * after the event is completed.
  * \param address destination address for which this event is running.
  * \param id unique eventid that was generated.
  * \return true on success
  */
  bool
  AddIpv4Event (Ipv4Address address, EventId id);
  /**
  * Clear up the entry from the map after the event is completed
  * \param address destination address for which this event is running.
  * \return true on success
  */
  bool
  DeleteIpv4Event (Ipv4Address address);
  /**
  * Force delete an update waiting for settling time to complete as a better update to
  * same destination was received.
  * \param address destination address for which this event is running.
  * \return true on success
  */
  bool
  AnyRunningEvent (Ipv4Address address);
  /**
  * Force delete an update waiting for settling time to complete as a better update to
  * same destination was received.
  * \param address destination address for which this event is running.
  * \return true on finding out that an event is already running for that destination address.
  */
  bool
  ForceDeleteIpv4Event (Ipv4Address address);
  /**
    * Get the EcentId associated with that address.
    * \param address destination address for which this event is running.
    * \return EventId on finding out an event is associated else return NULL.
    */
  EventId
  GetEventId (Ipv4Address address);

  /**
   * Get hold down time (time until an invalid route may be deleted)
   * \returns the hold down time
   */
  Time Getholddowntime () const
  {
    return m_holddownTime;
  }
  /**
   * Set hold down time (time until an invalid route may be deleted)
   * \param t the hold down time
   */
  void Setholddowntime (Time t)
  {
    m_holddownTime = t;
  }

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
  Ipv4Address BestNeighbor (Ipv4Address dst, Vector myPos);

  // ADD: 检查是否符合恢复模式的条件（有目的地地址&有邻居&没有可以使用贪婪的下一跳）
  bool MatchRecovery(Ipv4Address dst, Vector myPos);

private:
  // Fields
  /// an entry in the routing table.
  std::map<Ipv4Address, RoutingTableEntry> m_ipv4AddressEntry;
  /// an entry in the event table.
  std::map<Ipv4Address, EventId> m_ipv4Events;
  /// hold down time of an expired route
  Time m_holddownTime;

};
}
}
#endif /* MYPROTOCOL_RTABLE_H */
