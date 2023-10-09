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
#include "myprotocol4-rtable.h"
#include "ns3/simulator.h"
#include <iomanip>
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Myprotocol4RoutingTable");

// ADD:修改构造函数中的参数
namespace myprotocol4 {
RoutingTableEntry::RoutingTableEntry (uint16_t x,
                                      uint16_t y,
                                      uint16_t z,
                                      int16_t vx,
                                      int16_t vy,
                                      int16_t vz,
                                      uint16_t timestamp,
                                      Ipv4Address adress)
  : m_x(x),
    m_y(y),
    m_z(z),
    m_vx(vx),
    m_vy(vy),
    m_vz(vz),
    m_timestamp(timestamp),
    m_adress(adress)
{
}
RoutingTableEntry::~RoutingTableEntry ()
{
}


RoutingTable::RoutingTable ()
{
  // 每个表项的有效生存时间为maxIntervalTime
  m_entryLifeTime = 30;
}

bool
RoutingTable::LookupRoute (Ipv4Address id,
                           RoutingTableEntry & rt)
{
  Purge();
  if (m_positionTable.empty ())
    {
      return false;
    }
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_positionTable.find (id);
  if (i == m_positionTable.end ())
    {
      return false;
    }
  rt = i->second;
  return true;
}

bool
RoutingTable::DeleteRoute (Ipv4Address dst)
{
  if (m_positionTable.erase (dst) != 0)
    {
      // NS_LOG_DEBUG("Route erased");
      return true;
    }
  return false;
}

bool
RoutingTable::AddRoute (RoutingTableEntry & rt)
{
  std::pair<std::map<Ipv4Address, RoutingTableEntry>::iterator, bool> result = m_positionTable.insert (std::make_pair (
                                                                                                            rt.GetAdress (),rt));
  return result.second;
}

bool
RoutingTable::Update (RoutingTableEntry & rt)
{
  std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_positionTable.find (rt.GetAdress ());
  if (i == m_positionTable.end ())
    {
      AddRoute(rt);
    }
  i->second = rt;
  return true;
}

void
RoutingTableEntry::Print (Ptr<OutputStreamWrapper> stream) const
{
  *stream->GetStream () << std::setiosflags (std::ios::fixed) << m_x << "\t\t" << m_y << "\t\t" << m_z << "\t\t"
                        << m_vx << "\t\t" << m_vy << "\t\t" << m_vz << "\t\t" << m_timestamp << "\t\t" << m_adress << "\n";
}

void
RoutingTable::Print (Ptr<OutputStreamWrapper> stream) const
{
  *stream->GetStream () << "\n myprotocol Routing table\n" << "x\t\ty\t\tz\t\tvx\t\tvy\t\tvz\t\ttimestamp\t\tadress\n";
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_positionTable.begin (); i
       != m_positionTable.end (); ++i)
    {
      i->second.Print (stream);
    }
  *stream->GetStream () << "=====================neighbor table=======================\n";
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_neiborTable.begin (); i
       != m_neiborTable.end (); ++i)
    {
      i->second.Print (stream);
    }

  *stream->GetStream () << "\n";
}

// ADD:位置预测函数
Vector 
RoutingTable::PredictPosition(Ipv4Address id){
  RoutingTableEntry rt;
  if(!LookupRoute(id,rt)){
    std::cout<<"not find a valid routing entry!!!\n";
    return Vector(-1,-1,-1);
  }else{
    // 先获取该节点的速度、位置、时间戳
    uint16_t deltaTime = Simulator::Now ().ToInteger(Time::S) - rt.GetTimestamp();
    // std::cout<<"deltaTime = "<<deltaTime<<"; now = "<<Simulator::Now ().ToInteger(Time::S)<<"; timestamp = "<<rt.GetTimestamp()<<"\n";
    int16_t tempX = rt.GetX() + deltaTime * rt.GetVx();
    int16_t tempY = rt.GetY() + deltaTime * rt.GetVy();
    int16_t tempZ = rt.GetZ() + deltaTime * rt.GetVz();
    uint16_t newX = tempX > 0 ? tempX : 0;
    uint16_t newY = tempY > 0 ? tempY : 0;
    uint16_t newZ = tempZ > 0 ? tempZ : 0;
    uint16_t maxX = 1000;
    uint16_t maxY = 1000;
    uint16_t maxZ = 300;
    newX = newX > maxX ? maxX : newX;
    newY = newY > maxY ? maxY : newY;
    newZ = newZ > maxZ ? maxZ : newZ;
    return Vector(newX, newY, newZ);
  }
}

// ADD：筛选邻居节点
void 
RoutingTable::LookupNeighbor(std::map<Ipv4Address, RoutingTableEntry> & neighborTable, Vector myPos){
  // 传输范围250m
  uint16_t TransmissionRange = 250;
  m_neiborTable.clear();
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_positionTable.begin (); i != m_positionTable.end (); i++){
    if(i->first == Ipv4Address::GetLoopback () || i->first == Ipv4Address("10.1.1.255") || i->first == Ipv4Address("255.255.255.255")){
      continue;
    }
    RoutingTableEntry rt = i->second;
    Vector predictPos = PredictPosition(i->first);
    if(CalculateDistance (predictPos, Vector(-1,-1,-1)) == 0){
      continue;
    }
    double distance = CalculateDistance(predictPos, myPos);
    // 距离小于传输范围，可以认为是邻居
    if(distance <= TransmissionRange){
      neighborTable.insert(std::make_pair(i->first,i->second));
      m_neiborTable.insert(std::make_pair(i->first,i->second));
    }
  }
}


// ADD：实现贪婪寻找最优下一条路径，！！！dstPos：是经过预测后的目的地地址！！！
Ipv4Address 
RoutingTable::BestNeighbor (std::map<Ipv4Address, RoutingTableEntry> neighborTable, Vector dstPos, Vector myPos)
{
  if(CalculateDistance (dstPos, Vector(-1,-1,-1)) == 0){
    return Ipv4Address::GetZero ();
  }

  double initialDistance = CalculateDistance (dstPos, myPos);

  if (neighborTable.empty ())
    {
      std::cout<<"Neighbor table is empty!!!\n";
      NS_LOG_DEBUG ("Neighbor table is empty");
      return Ipv4Address::GetZero ();
    }     //if table is empty (no neighbours)
  Ipv4Address bestFoundID = neighborTable.begin ()->first;
  double bestFoundDistance = CalculateDistance (PredictPosition(neighborTable.begin ()->first), dstPos);

  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = neighborTable.begin (); i != neighborTable.end (); i++){
    if(bestFoundDistance > CalculateDistance (PredictPosition(i->first), dstPos)){
      bestFoundID = i->first;
      bestFoundDistance = CalculateDistance (PredictPosition(i->first), dstPos);
    }
  }  
  if(initialDistance > bestFoundDistance){
    return bestFoundID;
  }else{
    std::cout<<"There is no closer neighbor!!!\n";
    return Ipv4Address::GetZero (); //so it enters Recovery-mode
  }
}

// ADD：清理过期表项
void
RoutingTable::Purge(){
  if (m_positionTable.empty ())
  {
    return;
  }

  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_positionTable.begin (); i != m_positionTable.end (); ){
    if (m_entryLifeTime + i->second.GetTimestamp() <= Simulator::Now ().ToInteger(Time::S)){
      std::map<Ipv4Address, RoutingTableEntry>::iterator itmp = i;
      ++i;
      m_positionTable.erase (itmp);
    }else{
      ++i;
    }
  }
  return;
}

}
}
