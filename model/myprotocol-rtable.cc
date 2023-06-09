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
#include "myprotocol-rtable.h"
#include "ns3/simulator.h"
#include <iomanip>
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MyprotocolRoutingTable");

// ADD:修改构造函数中的参数
namespace myprotocol {
RoutingTableEntry::RoutingTableEntry (Ptr<NetDevice> dev,
                                      Ipv4Address dst,
                                      uint32_t seqNo,
                                      Ipv4InterfaceAddress iface,
                                      uint32_t hops,
                                      Ipv4Address nextHop,
                                      Time lifetime,
                                      Time SettlingTime,
                                      bool areChanged,
                                      uint16_t x,
                                      uint16_t y,
                                      uint16_t z,
                                      int16_t vx,
                                      int16_t vy,
                                      int16_t vz,
                                      uint16_t timestamp,
                                      Ipv4Address adress)
  : m_seqNo (seqNo),
    m_hops (hops),
    m_lifeTime (lifetime),
    m_iface (iface),
    m_flag (VALID),
    m_settlingTime (SettlingTime),
    m_entriesChanged (areChanged),
    m_x(x),
    m_y(y),
    m_z(z),
    m_vx(vx),
    m_vy(vy),
    m_vz(vz),
    m_timestamp(timestamp),
    m_adress(adress)
{
  m_ipv4Route = Create<Ipv4Route> ();
  m_ipv4Route->SetDestination (dst);
  m_ipv4Route->SetGateway (nextHop);
  m_ipv4Route->SetSource (m_iface.GetLocal ());
  m_ipv4Route->SetOutputDevice (dev);
}
RoutingTableEntry::~RoutingTableEntry ()
{
}
RoutingTable::RoutingTable ()
{
}

bool
RoutingTable::LookupRoute (Ipv4Address id,
                           RoutingTableEntry & rt)
{
  if (m_ipv4AddressEntry.empty ())
    {
      return false;
    }
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_ipv4AddressEntry.find (id);
  if (i == m_ipv4AddressEntry.end ())
    {
      return false;
    }
  rt = i->second;
  return true;
}

bool
RoutingTable::LookupRoute (Ipv4Address id,
                           RoutingTableEntry & rt,
                           bool forRouteInput)
{
  if (m_ipv4AddressEntry.empty ())
    {
      return false;
    }
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_ipv4AddressEntry.find (id);
  if (i == m_ipv4AddressEntry.end ())
    {
      return false;
    }
  if (forRouteInput == true && id == i->second.GetInterface ().GetBroadcast ())
    {
      return false;
    }
  rt = i->second;
  return true;
}

bool
RoutingTable::DeleteRoute (Ipv4Address dst)
{
  if (m_ipv4AddressEntry.erase (dst) != 0)
    {
      // NS_LOG_DEBUG("Route erased");
      return true;
    }
  return false;
}

uint32_t
RoutingTable::RoutingTableSize ()
{
  return m_ipv4AddressEntry.size ();
}

bool
RoutingTable::AddRoute (RoutingTableEntry & rt)
{
  std::pair<std::map<Ipv4Address, RoutingTableEntry>::iterator, bool> result = m_ipv4AddressEntry.insert (std::make_pair (
                                                                                                            rt.GetDestination (),rt));
  return result.second;
}

bool
RoutingTable::Update (RoutingTableEntry & rt)
{
  std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_ipv4AddressEntry.find (rt.GetDestination ());
  if (i == m_ipv4AddressEntry.end ())
    {
      return false;
    }
  i->second = rt;
  return true;
}

void
RoutingTable::DeleteAllRoutesFromInterface (Ipv4InterfaceAddress iface)
{
  if (m_ipv4AddressEntry.empty ())
    {
      return;
    }
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); )
    {
      if (i->second.GetInterface () == iface)
        {
          std::map<Ipv4Address, RoutingTableEntry>::iterator tmp = i;
          ++i;
          m_ipv4AddressEntry.erase (tmp);
        }
      else
        {
          ++i;
        }
    }
}

void
RoutingTable::GetListOfAllRoutes (std::map<Ipv4Address, RoutingTableEntry> & allRoutes)
{
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); ++i)
    {
      if (i->second.GetDestination () != Ipv4Address ("127.0.0.1") && i->second.GetFlag () == VALID)
        {
          allRoutes.insert (
            std::make_pair (i->first,i->second));
        }
    }
}

void
RoutingTable::GetListOfDestinationWithNextHop (Ipv4Address nextHop,
                                               std::map<Ipv4Address, RoutingTableEntry> & unreachable)
{
  unreachable.clear ();
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_ipv4AddressEntry.begin (); i
       != m_ipv4AddressEntry.end (); ++i)
    {
      if (i->second.GetNextHop () == nextHop)
        {
          unreachable.insert (std::make_pair (i->first,i->second));
        }
    }
}

void
RoutingTableEntry::Print (Ptr<OutputStreamWrapper> stream) const
{
  *stream->GetStream () << std::setiosflags (std::ios::fixed) << m_ipv4Route->GetDestination () << "\t\t" << m_ipv4Route->GetGateway () << "\t\t"
                        << m_iface.GetLocal () << "\t\t" << std::setiosflags (std::ios::left)
                        << std::setw (10) << m_hops << "\t" << std::setw (10) << m_seqNo << "\t"
                        << std::setprecision (3) << (Simulator::Now () - m_lifeTime).GetSeconds ()
                        << "s\t\t" << m_settlingTime.GetSeconds () << "s\t\t" << m_x << "\t\t" << m_y << "\t\t" << m_z << "\t\t"
                        << m_vx << "\t\t" << m_vy << "\t\t" << m_vz << "\t\t" << m_timestamp << "\t\t" << m_adress << "\n";
}

void
RoutingTable::Purge (std::map<Ipv4Address, RoutingTableEntry> & removedAddresses)
{
  if (m_ipv4AddressEntry.empty ())
    {
      return;
    }
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); )
    {
      std::map<Ipv4Address, RoutingTableEntry>::iterator itmp = i;
      if (i->second.GetLifeTime () > m_holddownTime && (i->second.GetHop () > 0))
        {
          for (std::map<Ipv4Address, RoutingTableEntry>::iterator j = m_ipv4AddressEntry.begin (); j != m_ipv4AddressEntry.end (); )
            {
              if ((j->second.GetNextHop () == i->second.GetDestination ()) && (i->second.GetHop () != j->second.GetHop ()))
                {
                  std::map<Ipv4Address, RoutingTableEntry>::iterator jtmp = j;
                  removedAddresses.insert (std::make_pair (j->first,j->second));
                  ++j;
                  m_ipv4AddressEntry.erase (jtmp);
                }
              else
                {
                  ++j;
                }
            }
          removedAddresses.insert (std::make_pair (i->first,i->second));
          ++i;
          m_ipv4AddressEntry.erase (itmp);
        }
      /** \todo Need to decide when to invalidate a route */
      /*          else if (i->second.GetLifeTime() > m_holddownTime)
       {
       ++i;
       itmp->second.SetFlag(INVALID);
       }*/
      else
        {
          ++i;
        }
    }
  return;
}

void
RoutingTable::Print (Ptr<OutputStreamWrapper> stream) const
{
  *stream->GetStream () << "\n myprotocol Routing table\n" << "Destination\t\tGateway\t\tInterface\t\tHopCount\t\tSeqNum\t\tLifeTime\t\tSettlingTime\t\tx\t\ty\t\tz\t\tvx\t\tvy\t\tvz\t\ttimestamp\t\tadress\n";
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_ipv4AddressEntry.begin (); i
       != m_ipv4AddressEntry.end (); ++i)
    {
      i->second.Print (stream);
    }
  *stream->GetStream () << "\n";
}

bool
RoutingTable::AddIpv4Event (Ipv4Address address,
                            EventId id)
{
  std::pair<std::map<Ipv4Address, EventId>::iterator, bool> result = m_ipv4Events.insert (std::make_pair (address,id));
  return result.second;
}

bool
RoutingTable::AnyRunningEvent (Ipv4Address address)
{
  EventId event;
  std::map<Ipv4Address, EventId>::const_iterator i = m_ipv4Events.find (address);
  if (m_ipv4Events.empty ())
    {
      return false;
    }
  if (i == m_ipv4Events.end ())
    {
      return false;
    }
  event = i->second;
  if (event.IsRunning ())
    {
      return true;
    }
  else
    {
      return false;
    }
}

bool
RoutingTable::ForceDeleteIpv4Event (Ipv4Address address)
{
  EventId event;
  std::map<Ipv4Address, EventId>::const_iterator i = m_ipv4Events.find (address);
  if (m_ipv4Events.empty () || i == m_ipv4Events.end ())
    {
      return false;
    }
  event = i->second;
  Simulator::Cancel (event);
  m_ipv4Events.erase (address);
  return true;
}

bool
RoutingTable::DeleteIpv4Event (Ipv4Address address)
{
  EventId event;
  std::map<Ipv4Address, EventId>::const_iterator i = m_ipv4Events.find (address);
  if (m_ipv4Events.empty () || i == m_ipv4Events.end ())
    {
      return false;
    }
  event = i->second;
  if (event.IsRunning ())
    {
      return false;
    }
  if (event.IsExpired ())
    {
      event.Cancel ();
      m_ipv4Events.erase (address);
      return true;
    }
  else
    {
      m_ipv4Events.erase (address);
      return true;
    }
}

EventId
RoutingTable::GetEventId (Ipv4Address address)
{
  std::map <Ipv4Address, EventId>::const_iterator i = m_ipv4Events.find (address);
  if (m_ipv4Events.empty () || i == m_ipv4Events.end ())
    {
      return EventId ();
    }
  else
    {
      return i->second;
    }
}

// ADD:位置预测函数
Vector 
RoutingTable::PredictPosition(Ipv4Address id){
  RoutingTableEntry rt;
  if(!LookupRoute(id,rt)){
    std::cout<<"not find a valid routing entry!!!\n";
    return Vector(0,0,0);
  }else{
    // 先获取该节点的速度、位置、时间戳
    uint16_t deltaTime = rt.GetTimestamp() - Simulator::Now ().ToInteger(Time::S);
    uint16_t newX = rt.GetX() + deltaTime * rt.GetVx();
    uint16_t newY = rt.GetY() + deltaTime * rt.GetVy();
    uint16_t newZ = rt.GetZ() + deltaTime * rt.GetVz();
    return Vector(newX, newY, newZ);
  }
}

// ADD：筛选邻居节点
void 
RoutingTable::LookupNeighbor(std::map<Ipv4Address, RoutingTableEntry> & neighborTable, Vector myPos){
  // 传输范围250m
  uint16_t TransmissionRange = 250;
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); i++){
    Vector predictPos = PredictPosition(i->first);
    double distance = CalculateDistance(predictPos, myPos);
    // 距离小于传输范围，可以认为是邻居
    if(distance <= TransmissionRange){
      neighborTable.insert(std::make_pair(i->first,i->second));
    }
  }
}

// ADD：实现贪婪寻找最优下一条路径
/**
 * \brief Gets next hop according to GPSR protocol
 * \param position the position of the destination node
 * \param nodePos the position of the node that has the packet
 * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
 */
Ipv4Address 
RoutingTable::BestNeighbor (Ipv4Address dst, Vector myPos)
{
  std::map<Ipv4Address, RoutingTableEntry> neighborTable;
  LookupNeighbor(neighborTable, myPos);
  Vector predictDstPos = PredictPosition(dst);

  double initialDistance = CalculateDistance (predictDstPos, myPos);

  if (neighborTable.empty ())
    {
      std::cout<<"BestNeighbor table is empty!!!\n";
      NS_LOG_DEBUG ("BestNeighbor table is empty");
      return Ipv4Address::GetZero ();
    }     //if table is empty (no neighbours)

  Ipv4Address bestFoundID = neighborTable.begin ()->first;
  double bestFoundDistance = CalculateDistance (PredictPosition(neighborTable.begin ()->first), predictDstPos);

  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = neighborTable.begin (); i != neighborTable.end (); i++){
    if(bestFoundDistance > CalculateDistance (PredictPosition(i->first), predictDstPos)){
      bestFoundID = i->first;
      bestFoundDistance = CalculateDistance (PredictPosition(i->first), predictDstPos);
    }
  }  
  if(initialDistance > bestFoundDistance){
    return bestFoundID;
  }else{
    return Ipv4Address::GetZero (); //so it enters Recovery-mode
  }
}
}
}
