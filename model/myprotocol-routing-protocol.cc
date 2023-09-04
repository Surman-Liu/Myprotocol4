/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 Hemanth Narra, Yufei Cheng
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
 * Author: Yufei Cheng   <yfcheng@ittc.ku.edu>
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

#include "myprotocol-routing-protocol.h"
#include "ns3/log.h"
#include "ns3/inet-socket-address.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/wifi-net-device.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/icmpv4.h"
#include "ns3/udp-header.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MyprotocolRoutingProtocol");

namespace myprotocol {

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

/// UDP Port for myprotocol control traffic
const uint32_t RoutingProtocol::MYPROTOCOL_PORT = 269;

TypeId
RoutingProtocol::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::myprotocol::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .SetGroupName ("Myprotocol")
    .AddConstructor<RoutingProtocol> ()
    .AddAttribute ("PeriodicUpdateInterval","Periodic interval between exchange of full routing tables among nodes. ",
                   TimeValue (Seconds (10)),
                   MakeTimeAccessor (&RoutingProtocol::m_periodicUpdateInterval),
                   MakeTimeChecker ())
    .AddAttribute ("CheckChangeInterval","Check interval between compare speed and thata with thire threshold. ",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&RoutingProtocol::m_checkChangeInterval),
                   MakeTimeChecker ())
    .AddAttribute ("EnableAdaptiveUpdate","Enables adaptive update node information. ",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RoutingProtocol::SetEnableAdaptiveUpdate,
                                        &RoutingProtocol::GetEnableAdaptiveUpdate),
                   MakeBooleanChecker ());            
  return tid;
}

int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

// ADD：在此处初始化与id-cache相关的变量
RoutingProtocol::RoutingProtocol ()
  : m_routingTable (),
    m_thetaThreshold(20),         //角度变化超过10度发送更新
    m_speedThreshold(20),         //速度变化超过10m/s发送更新
    m_netDiameter (15),                                    //最大跳数，1000*根号二/250m = 6hops
    m_nodeTraversalTime (MilliSeconds (40)),               //一跳的传播速度，250m / 299792458m/s = 
    m_netTraversalTime (Time ((2 * m_netDiameter) * m_nodeTraversalTime)),
    m_pathDiscoveryTime ( Time (2 * m_netTraversalTime)),
    m_lastSendTime(0),
    m_ifChangeLastTime(false),
    m_maxIntervalTime(30),
    m_idCache(m_pathDiscoveryTime),            // 每个生命周期是2.4s
    m_maxQueueLen (64),
    m_maxQueueTime (Seconds (30)),
    m_queue (m_maxQueueLen, m_maxQueueTime),
    m_periodicUpdateTimer (Timer::CANCEL_ON_DESTROY),
    m_checkChangeTimer(Timer::CANCEL_ON_DESTROY)
{
  m_recordInformation.speed = 0;
  m_recordInformation.thetaXY = 0;
  m_recordInformation.thetaZ = 0;
  m_sendInformation.speed = 0;
  m_sendInformation.thetaXY = 0;
  m_sendInformation.thetaZ = 0;
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
}

RoutingProtocol::~RoutingProtocol ()
{
}

void
RoutingProtocol::DoDispose ()
{
  m_ipv4 = 0;
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::iterator iter = m_socketAddresses.begin (); iter
       != m_socketAddresses.end (); iter++)
    {
      iter->first->Close ();
    }
  m_socketAddresses.clear ();
  Ipv4RoutingProtocol::DoDispose ();
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
  *stream->GetStream () << "Node: " << m_ipv4->GetObject<Node> ()->GetId ()
                        << ", Time: " << Now ().As (unit)
                        << ", Local time: " << GetObject<Node> ()->GetLocalTime ().As (unit)
                        << ", MYPROTOCOL Routing table" << std::endl;

  m_routingTable.Print (stream);
  *stream->GetStream () << std::endl;
}

void
RoutingProtocol::Start ()
{
  m_scb = MakeCallback (&RoutingProtocol::Send,this);
  m_ecb = MakeCallback (&RoutingProtocol::Drop,this);
  SendPeriodicUpdate();
  if(!m_enableAdaptiveUpdate){
    m_periodicUpdateTimer.SetFunction (&RoutingProtocol::SendPeriodicUpdate,this);
    m_periodicUpdateTimer.Schedule (MicroSeconds (m_uniformRandomVariable->GetInteger (0,1000)));
  }else{
    m_checkChangeTimer.SetFunction (&RoutingProtocol::CheckChange,this);
    m_checkChangeTimer.Schedule (MicroSeconds (m_uniformRandomVariable->GetInteger (0,1000)));
  }
}

// 既可以转发控制包，也可以转发数据包，但是都是自己发出的
Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p,
                              const Ipv4Header &header,
                              Ptr<NetDevice> oif,
                              Socket::SocketErrno &sockerr)
{
  NS_LOG_FUNCTION (this << header << (oif ? oif->GetIfIndex () : 0));

  if (!p)
    {
      return LoopbackRoute (header,oif);
    }
  if (m_socketAddresses.empty ())
    {
      sockerr = Socket::ERROR_NOROUTETOHOST;
      NS_LOG_LOGIC ("No myprotocol interfaces");
      Ptr<Ipv4Route> route;
      return route;
    }

  Ptr<Ipv4Route> route = Create<Ipv4Route> ();
  Ipv4Address dst = header.GetDestination ();

  // ADD：发送自己的数据包和控制包
  // 如果目的地是广播的话，那说明这是一个控制包，直接广播即可。
  if(dst == m_ipv4->GetAddress (1, 0).GetBroadcast ()){
    route->SetDestination(m_ipv4->GetAddress (1, 0).GetBroadcast ());
    route->SetGateway(m_ipv4->GetAddress (1, 0).GetBroadcast ());
    route->SetSource(m_ipv4->GetAddress (1, 0).GetLocal ());
    route->SetOutputDevice(m_ipv4->GetNetDevice (1));
    return route;
  }else{
    // 目的地是单播，则是一个数据包，需要添加贪婪转发状态的包头，再进行贪婪转发。
    DataHeader dataHeader;
    dataHeader.SetDstPosx(0);
    dataHeader.SetDstPosy(0);
    dataHeader.SetDstPosz(0);
    dataHeader.SetDstVelx(0);
    dataHeader.SetDstVely(0);
    dataHeader.SetDstVelz(0);
    dataHeader.SetDstSign(0);
    dataHeader.SetDstTimestamp(0);
    dataHeader.SetRecPosx(0);
    dataHeader.SetRecPosy(0);
    dataHeader.SetRecPosz(0);
    dataHeader.SetInRec(0);
    dataHeader.SetUid(p->GetUid ());
    dataHeader.SetHop(1);

    RoutingTableEntry rt;
    if(m_routingTable.LookupRoute(dst,rt)){
      // 将rt中关于目的地的位置、速度和时间戳写进dataHeader
      uint16_t dstSign = SetRightVelocity(rt.GetVx(), rt.GetVy(), rt.GetVz());
      dataHeader.SetDstPosx(rt.GetX());
      dataHeader.SetDstPosy(rt.GetY());
      dataHeader.SetDstPosz(rt.GetZ());
      dataHeader.SetDstVelx(abs(rt.GetVx()));
      dataHeader.SetDstVely(abs(rt.GetVy()));
      dataHeader.SetDstVelz(abs(rt.GetVz()));
      dataHeader.SetDstSign(dstSign);
      dataHeader.SetDstTimestamp(rt.GetTimestamp());

      // 30s，和最大间隔更新时间一样
      m_routingTable.Purge();

      Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
      Vector myPos = MM->GetPosition();
      std::map<Ipv4Address, RoutingTableEntry> neighborTable;
      m_routingTable.LookupNeighbor(neighborTable, myPos);

      // 有邻居也有目的地的位置，则一定可以找到转发出去的下一跳
      Vector dstPos = m_routingTable.PredictPosition(dst);

      Ipv4Address nexthop = m_routingTable.BestNeighbor(neighborTable, dstPos, myPos);
      // 数据包找到了合适的下一跳
      if(nexthop != Ipv4Address::GetZero ()){
        p->AddHeader(dataHeader);
        route->SetDestination(dst);
        route->SetGateway(nexthop);
        route->SetSource (m_ipv4->GetAddress (1, 0).GetLocal ());
        route->SetOutputDevice (m_ipv4->GetNetDevice (1));
        return route;
      }
      else{
        // 没有找到合适的下一跳,符合恢复转发的条件（有目的地位置，有可转发邻居）
        dataHeader.SetRecPosx((uint16_t)myPos.x);
        dataHeader.SetRecPosy((uint16_t)myPos.y);
        dataHeader.SetRecPosz((uint16_t)myPos.z);
        dataHeader.SetInRec(1);
        p->AddHeader(dataHeader);
        // 恢复模式获得下一跳
        nexthop = RecoveryMode (neighborTable);
        Ptr<Ipv4Route> route = Create<Ipv4Route> ();
        route->SetDestination (dst);
        route->SetGateway (nexthop);
        route->SetSource (m_ipv4->GetAddress (1, 0).GetLocal ());
        route->SetOutputDevice (m_ipv4->GetNetDevice (1));  
        return route;
      }
    }else{
      p->AddHeader(dataHeader);
      // 没有目的地的地址/没有邻居
      return LoopbackRoute (header,oif);
    }
  }
}

// 只转发数据包,并且会在规定的数据包长度前面再加上8个字节，但是他只是转发别人的数据包，当自己要主动发出数据包的时候，不调用这个函数。
bool
RoutingProtocol::RouteInput (Ptr<const Packet> p,
                             const Ipv4Header &header,
                             Ptr<const NetDevice> idev,
                             UnicastForwardCallback ucb,
                             MulticastForwardCallback mcb,
                             LocalDeliverCallback lcb,
                             ErrorCallback ecb)
{  
  if (m_socketAddresses.empty ())
    {
      NS_LOG_DEBUG ("No myprotocol interfaces");
      return false;
    }

  NS_ASSERT (m_ipv4 != 0);
  // Check if input device supports IP
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  int32_t iif = m_ipv4->GetInterfaceForDevice (idev);

  Ipv4Address dst = header.GetDestination ();
  Ipv4Address origin = header.GetSource ();

  // myprotocol is not a multicast routing protocol
  // IsMulticast：true only if address is in the range 224.0.0.0 - 239.255.255.255，这里会筛掉广播的地址
  if (dst.IsMulticast ())
    {
      return false;
    }

  // Deferred route request，收到回环的数据包
  // todo:可以使用队列。
  if (idev == m_lo)
    {
      // 加入队列
      DeferredRouteOutput (p, header, ucb, ecb);
      return true;
    }

  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (origin == iface.GetLocal ())
        {
          return true;
        }
    }

  // LOCAL DELIVARY TO DSDV INTERFACES
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin (); j
       != m_socketAddresses.end (); ++j)
  {
    Ipv4InterfaceAddress iface = j->second;
    if (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()) == iif)
      {
        if (dst == iface.GetBroadcast () || dst.IsBroadcast ())
          {
            Ptr<Packet> packet = p->Copy ();
            if (lcb.IsNull () == false)
              {
                NS_LOG_LOGIC ("Broadcast local delivery to " << iface.GetLocal ());
                lcb (p, header, iif);
                // Fall through to additional processing
              }
            else
              {
                NS_LOG_ERROR ("Unable to deliver packet locally due to null callback " << p->GetUid () << " from " << origin);
                ecb (p, header, Socket::ERROR_NOROUTETOHOST);
              }
            if (header.GetTtl () > 1)
              {
                NS_LOG_LOGIC ("Forward broadcast. TTL " << (uint16_t) header.GetTtl ());
                RoutingTableEntry toBroadcast;
                if (m_routingTable.LookupRoute (dst,toBroadcast))
                  {
                    Ptr<Ipv4Route> route = Create<Ipv4Route> ();
                    route->SetDestination(m_ipv4->GetAddress (1, 0).GetBroadcast ());
                    route->SetGateway(m_ipv4->GetAddress (1, 0).GetBroadcast ());
                    route->SetSource(m_ipv4->GetAddress (1, 0).GetLocal ());
                    route->SetOutputDevice(m_ipv4->GetNetDevice (1));
                    ucb (route,packet,header);
                  }
                else
                  {
                    NS_LOG_DEBUG ("No route to forward. Drop packet " << p->GetUid ());
                  }
              }
            return true;
          }
      }
  }

  // 自己就是数据包的目的地，则只需要本地转发
  if (m_ipv4->IsDestinationAddress (dst, iif))
    {
      Ptr<Packet> packet = p->Copy ();
      if (lcb.IsNull () == false)
        {
          DataHeader dataHeader;
          packet->RemoveHeader(dataHeader);
          NS_LOG_LOGIC ("Unicast local delivery to " << dst);
          lcb (packet, header, iif);
        }
      else
        {
          NS_LOG_ERROR ("Unable to deliver packet locally due to null callback " << p->GetUid () << " from " << origin);
          ecb (p, header, Socket::ERROR_NOROUTETOHOST);
        }
      return true;
    }

  // Check if input device supports IP forwarding
  if (m_ipv4->IsForwarding (iif) == false)
    {
      NS_LOG_LOGIC ("Forwarding disabled for this interface");
      ecb (p, header, Socket::ERROR_NOROUTETOHOST);
      return true;
    }
  
  // 贪婪转发 or 恢复模式 or 丢弃
  return Forwarding (p, header, ucb, ecb);
}

void
RoutingProtocol::DeferredRouteOutput (Ptr<const Packet> p, const Ipv4Header & header,
                                      UnicastForwardCallback ucb, ErrorCallback ecb)
{
  NS_ASSERT (p != 0 && p != Ptr<Packet> ());

  QueueEntry newEntry (p, header, ucb, ecb);
  m_queue.Enqueue (newEntry);
}

/// ADD：If route exists and valid, forward packet.
bool 
RoutingProtocol::Forwarding (Ptr<const Packet> packet, const Ipv4Header & header, UnicastForwardCallback ucb, ErrorCallback ecb){
  PacketMetadata::ItemIterator i = packet->BeginItem();
  PacketMetadata::Item item = i.Next ();
  TypeId id = item.tid;
  Ptr<Packet> p = packet->Copy ();
  Icmpv4Header icmpv4Header;
  UdpHeader udpHeader;
  
  // icmpv4的包
  if(id == icmpv4Header.GetTypeId()){
    std::cout<<"this is not udp packet!!!\n";
    p->RemoveHeader(icmpv4Header);
  }else{
    p->RemoveHeader(udpHeader);
  }
  
  DataHeader dataHeader;
  p->RemoveHeader(dataHeader);

  dataHeader.SetHop(dataHeader.GetHop() + 1);

  Vector DstPosition;
  Vector DstVelocity;
  uint16_t DstTimestamp = 0;
  Vector RecPosition;
  uint16_t inRec = 0;

  DstPosition.x = dataHeader.GetDstPosx ();
  DstPosition.y = dataHeader.GetDstPosy ();
  DstPosition.z = dataHeader.GetDstPosz ();
  DstVelocity = GetRightVelocity(dataHeader.GetDstVelx(),dataHeader.GetDstVely(),dataHeader.GetDstVelz(),dataHeader.GetDstSign());
  DstTimestamp = dataHeader.GetDstTimestamp();
  RecPosition.x = dataHeader.GetRecPosx ();
  RecPosition.y = dataHeader.GetRecPosy ();
  RecPosition.z = dataHeader.GetRecPosz ();
  inRec = dataHeader.GetInRec ();

  Ipv4Address dst = header.GetDestination ();

  m_routingTable.Purge();

  RoutingTableEntry rt;
  if(m_routingTable.LookupRoute(dst,rt)){
    // 更新目的地的最近地址
    if(rt.GetTimestamp() > DstTimestamp){
      // 该节点的目的地地址更新
      uint16_t sign = SetRightVelocity(rt.GetVx(),rt.GetVy(),rt.GetVz());
      dataHeader.SetDstPosx(rt.GetX());
      dataHeader.SetDstPosy(rt.GetY());
      dataHeader.SetDstPosz(rt.GetZ());
      dataHeader.SetDstVelx(abs(rt.GetVx()));
      dataHeader.SetDstVely(abs(rt.GetVy()));
      dataHeader.SetDstVelz(abs(rt.GetVz()));
      dataHeader.SetDstSign(sign);
      dataHeader.SetDstTimestamp(rt.GetTimestamp());
      // 修改用于预测位置的变量
      DstPosition.x = rt.GetX();
      DstPosition.y = rt.GetY();
      DstPosition.z = rt.GetZ();
      DstVelocity.x = rt.GetVx();
      DstVelocity.y = rt.GetVy();
      DstVelocity.z = rt.GetVz();
      DstTimestamp = rt.GetTimestamp();
    }else if(rt.GetTimestamp() < DstTimestamp){
      // 包头中的信息更新，则更新位置表
      RoutingTableEntry newRt (DstPosition.x, DstPosition.y, DstPosition.z,
                               DstVelocity.x, DstVelocity.y, DstVelocity.z,
                               DstTimestamp, dst);
      m_routingTable.Update(newRt);                         
    }
  }else{
    // 位置表中本来没有dst的信息，添加
    RoutingTableEntry newRt (DstPosition.x, DstPosition.y, DstPosition.z,
                             DstVelocity.x, DstVelocity.y, DstVelocity.z,
                             DstTimestamp, dst);
    m_routingTable.AddRoute(newRt);                           
  }

  Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
  Vector myPos = MM->GetPosition();

  // 预测目的地现在的位置
  Vector predictDst = m_routingTable.PredictPosition(dst);
  std::map<Ipv4Address, RoutingTableEntry> neighborTable;
  m_routingTable.LookupNeighbor(neighborTable, myPos);
   
  if(inRec == 1 && CalculateDistance (myPos, predictDst) < CalculateDistance (RecPosition, predictDst)){
    inRec = 0;
    dataHeader.SetInRec(0);
  }
  // 如果数据包本身就是恢复模式，并且该节点到目的地的距离比进入恢复模式到目的地的距离更远，则继续恢复模式
  if(inRec == 1){
    p->AddHeader (dataHeader);
    if(id == icmpv4Header.GetTypeId()){
      p->AddHeader(icmpv4Header);
    }else{
      p->AddHeader(udpHeader);
    }
    // 恢复模式
    Ipv4Address nextHop = RecoveryMode (neighborTable);
    Ptr<Ipv4Route> route = Create<Ipv4Route> ();
    route->SetDestination (dst);
    route->SetSource (header.GetSource ());
    route->SetGateway (nextHop);
    route->SetOutputDevice (m_ipv4->GetNetDevice (1)); 
    ucb (route, p, header); 
    return true;
    // return false;
  }else{     //贪婪模式
    Ipv4Address nextHop = m_routingTable.BestNeighbor (neighborTable, predictDst, myPos);
    if (nextHop != Ipv4Address::GetZero ())
    {
      p->AddHeader (dataHeader);  
      if(id == icmpv4Header.GetTypeId()){
        p->AddHeader(icmpv4Header);
      }else{
        p->AddHeader(udpHeader);
      }        
      Ptr<Ipv4Route> route = Create<Ipv4Route> ();
      route->SetDestination (dst);
      route->SetSource (header.GetSource ());
      route->SetGateway (nextHop);
      route->SetOutputDevice (m_ipv4->GetNetDevice (1));
      ucb (route, p, header);
      return true;
    }else{       // 如果贪婪转发没有找到合适的下一跳
      inRec = 1;
      dataHeader.SetInRec(1);
      dataHeader.SetRecPosx ((uint16_t)myPos.x);
      dataHeader.SetRecPosy ((uint16_t)myPos.y); 
      dataHeader.SetRecPosz ((uint16_t)myPos.z);
      p->AddHeader (dataHeader);
      if(id == icmpv4Header.GetTypeId()){
        p->AddHeader(icmpv4Header);
      }else{
        p->AddHeader(udpHeader);
      }
      // 恢复模式
      nextHop = RecoveryMode (neighborTable);
      Ptr<Ipv4Route> route = Create<Ipv4Route> ();
      route->SetDestination (dst);
      route->SetSource (header.GetSource ());
      route->SetGateway (nextHop);
      route->SetOutputDevice (m_ipv4->GetNetDevice (1));  
      ucb (route, p, header);
      return true;
      // return false;
    }
  }
}

// ADD：贪婪转发
Ipv4Address 
RoutingProtocol::RecoveryMode(std::map<Ipv4Address, RoutingTableEntry> & neighborTable){
  std::map<Ipv4Address, RoutingTableEntry>::iterator i = neighborTable.begin();

  // 随机选择一个邻居节点
  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
  uint16_t random = var->GetInteger(0,neighborTable.size());
  for (; random > 0 ; random--)
    {
        i++;
    }
  return i->first;
}

// 当RoutOutout没有找到合适的下一跳路由的时候会调用这个函数
// 返回一个gateway为自己，netdevice = m_lo = m_ipv4->GetNetDevice (0)的路由，该路由可以被RouteInput函数收到，
// RouteInput会对数据包进行判断是不是从回环地址收到的，如果是的话则调用DeferredRouteOutput()
Ptr<Ipv4Route>
RoutingProtocol::LoopbackRoute (const Ipv4Header & hdr, Ptr<NetDevice> oif) const
{
  NS_ASSERT (m_lo != 0);
  Ptr<Ipv4Route> rt = Create<Ipv4Route> ();
  rt->SetDestination (hdr.GetDestination ());

  std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin ();
  if (oif)
    {
      // Iterate to find an address on the oif device
      for (j = m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
        {
          Ipv4Address addr = j->second.GetLocal ();
          int32_t interface = m_ipv4->GetInterfaceForAddress (addr);
          if (oif == m_ipv4->GetNetDevice (static_cast<uint32_t> (interface)))
            {
              rt->SetSource (addr);
              break;
            }
        }
    }
  else
    {
      rt->SetSource (j->second.GetLocal ());
    }
  rt->SetGateway (Ipv4Address ("127.0.0.1"));
  rt->SetOutputDevice (m_lo);
  return rt;
}

// 只会处理控制包,即MyprotocolHeader
void
RoutingProtocol::RecvMyprotocol (Ptr<Socket> socket)
{
  Address sourceAddress;
  // RecvFrom中参数的类型是Address
  Ptr<Packet> packet = socket->RecvFrom (sourceAddress);

  MyprotocolHeader myprotocolHeader;
  packet->RemoveHeader (myprotocolHeader);

  Ipv4Address source = myprotocolHeader.GetMyadress();         //是哪个节点的位置信息

  // ADD:检查是否已经转发过，如果是则丢弃。
  if (m_idCache.IsDuplicate (source, myprotocolHeader.GetTimestamp()))
  {
    return;
  }

  m_routingTable.Purge();

  // 获取带符号的速度信息
  Vector velocity = GetRightVelocity(myprotocolHeader.GetVx(), myprotocolHeader.GetVy(), myprotocolHeader.GetVz(), myprotocolHeader.GetSign());
  RoutingTableEntry newEntry (
    myprotocolHeader.GetX(),
    myprotocolHeader.GetY(),
    myprotocolHeader.GetZ(),
    velocity.x,
    velocity.y,
    velocity.z,
    myprotocolHeader.GetTimestamp(),
    myprotocolHeader.GetMyadress()
  );
  m_routingTable.Update(newEntry);

  // 收到位置更新之后检查queue中是否有待发送的数据包
  // 更新了位置表之后，查看是否有数据包可以从queue中发送
  if(m_queue.Find(myprotocolHeader.GetMyadress())){
    // 如果队列中有目的地为source的数据包
    DataHeader dataHeader;
    // 将rt中关于目的地的位置、速度和时间戳写进dataHeader
    dataHeader.SetDstPosx(myprotocolHeader.GetX());
    dataHeader.SetDstPosy(myprotocolHeader.GetY());
    dataHeader.SetDstPosz(myprotocolHeader.GetZ());
    dataHeader.SetDstVelx(myprotocolHeader.GetVx());
    dataHeader.SetDstVely(myprotocolHeader.GetVy());
    dataHeader.SetDstVelz(myprotocolHeader.GetVz());
    dataHeader.SetDstSign(myprotocolHeader.GetSign());
    dataHeader.SetDstTimestamp(myprotocolHeader.GetTimestamp());

    //更新邻居表，以便打印路由 
    Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
    Vector myPos = MM->GetPosition();
    std::map<Ipv4Address, RoutingTableEntry> neighborTable;
    m_routingTable.LookupNeighbor(neighborTable, myPos);

    // 查找route
    Ptr<Ipv4Route> route = Create<Ipv4Route> ();

    // 根据预测的目的地位置，选择最优的下一跳
    Vector dstPos = m_routingTable.PredictPosition(myprotocolHeader.GetMyadress());
    Ipv4Address nexthop = m_routingTable.BestNeighbor(neighborTable, dstPos, myPos);
    // 数据包找到了合适的下一跳
    if(nexthop != Ipv4Address::GetZero ()){
      dataHeader.SetRecPosx(0);
      dataHeader.SetRecPosy(0);
      dataHeader.SetRecPosz(0);
      dataHeader.SetInRec(0);

      route->SetDestination(myprotocolHeader.GetMyadress());
      route->SetGateway(nexthop);
      route->SetSource (m_ipv4->GetAddress (1, 0).GetLocal ()); 
      route->SetOutputDevice (m_ipv4->GetNetDevice (1));
    }
    else{
      dataHeader.SetRecPosx((uint16_t)myPos.x);
      dataHeader.SetRecPosy((uint16_t)myPos.y);
      dataHeader.SetRecPosz((uint16_t)myPos.z);
      dataHeader.SetInRec(1);
      // 恢复模式获得下一跳
      Ipv4Address nexthop = RecoveryMode (neighborTable);
      route->SetDestination (myprotocolHeader.GetMyadress());
      route->SetGateway (nexthop);
      route->SetSource (m_ipv4->GetAddress (1, 0).GetLocal ()); 
      route->SetOutputDevice (m_ipv4->GetNetDevice (1));  
    }
    SendPacketFromQueue(myprotocolHeader.GetMyadress(), route, dataHeader);
  }
  
  Ptr<Packet> p = Create<Packet> ();
  p->AddHeader(myprotocolHeader);
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin (); j
       != m_socketAddresses.end (); ++j)
  {
    Ptr<Socket> socket = j->first;
    Ipv4InterfaceAddress iface = j->second;
    Ipv4Address destination;
    if (iface.GetMask () == Ipv4Mask::GetOnes ())
      {
        destination = Ipv4Address ("255.255.255.255");
      }
    else
      {
        destination = iface.GetBroadcast ();
      }
    socket->SendTo (p, 0, InetSocketAddress (destination, MYPROTOCOL_PORT));
  }
}

// ADD: 计算运动信息
void
RoutingProtocol::CalculateInformation (struct Information &information){
  Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
  Vector myVel = MM->GetVelocity();
  int16_t vx = (int16_t)myVel.x;
  int16_t vy = (int16_t)myVel.y;
  int16_t vz = (int16_t)myVel.z;

  // speed
  information.speed = sqrt(vx*vx + vy*vy + vz*vz);
  // thetaXY，[0,2π]
  if(vx == 0){
    if(vy == 0){
      information.thetaXY = 0;
    }else if(vy > 0){
      information.thetaXY = M_PI / 2;
    }else{
      information.thetaXY = M_PI * 3 / 2;
    }
  }else if(vx > 0){
    if(vy == 0){
      information.thetaXY = 0;
    }else if(vy > 0){
      information.thetaXY = atan(vy / vx);
    }else{
      information.thetaXY = 2 * M_PI + atan(vy / vx);
    }
  }else{
    if(vy == 0){
      information.thetaXY = M_PI;
    }else{
      information.thetaXY = M_PI + atan(vy / vx);
    }
  }
  // thetaZ，[π/2,-π/2]
  float vxy = sqrt(vx*vx + vy*vy);
  if(vxy == 0){
    if(vz > 0){
      information.thetaZ = M_PI / 2;
    }else if(vz < 0){
      information.thetaZ = - M_PI / 2;
    }else{
      information.thetaZ = 0;
    }
  }else{
    information.thetaZ = atan(vz / vxy);
  }
}

// ADD：定期检查速度、方向变化
void
RoutingProtocol::CheckChange ()
{
  struct Information information;
  // 计算当前信息
  CalculateInformation(information);
  // 计算差值
  float recordDeltaSpeed = fabs(information.speed - m_recordInformation.speed);
  float recordDeltaThetaXY = fabs(information.thetaXY - m_recordInformation.thetaXY);
  float recordDeltaThetaZ = fabs(information.thetaZ - m_recordInformation.thetaZ);
  // 将角度阈值变成弧度
  float thetaThreshold = m_thetaThreshold * M_PI / 180;
  // 变化超出阈值，重新发送更新控制包
  // 1. 超出阈值就更新
  if(recordDeltaSpeed > m_speedThreshold || recordDeltaThetaXY > thetaThreshold || recordDeltaThetaZ > thetaThreshold){
    // 将本次更新的速度、方向记录
    m_recordInformation = information;
    SendPeriodicUpdate();
  }
  // 如果超过最大间隔时间没有发送更新包，则发送
  if(Simulator::Now ().ToInteger(Time::S) - m_lastSendTime > m_maxIntervalTime){
    m_recordInformation = information;
    SendPeriodicUpdate();
  }
  // 2. 连续变化不更新
  // if(recordDeltaSpeed > m_speedThreshold || recordDeltaThetaXY > thetaThreshold || recordDeltaThetaZ > thetaThreshold){
  //   // 将本次更新的速度、方向记录
  //   m_recordInformation = information;
  //   // 如果不是连续时间
  //   if(m_ifChangeLastTime == false){
  //     SendPeriodicUpdate();
  //     m_sendInformation = information;
  //   }
  //   m_ifChangeLastTime = true;
  // }else{
  //   m_ifChangeLastTime = false;
  //   // case：如果信息经过了连续变化终于趋向稳定，但是平静后的第一次检测却没有超过阈值，则不能及时发送最新的位置
  //   float sendDeltaSpeed = fabs(information.speed - m_sendInformation.speed);
  //   float sendDeltaThetaXY = fabs(information.thetaXY - m_sendInformation.thetaXY);
  //   float sendDeltaThetaZ = fabs(information.thetaZ - m_sendInformation.thetaZ);
  //   if(sendDeltaSpeed > m_speedThreshold || sendDeltaThetaXY > thetaThreshold || sendDeltaThetaZ > thetaThreshold){
  //     // 虽然是false，但是已经很久没有发送新的控制包了，说明经过了连续变化，但是这不算true
  //     SendPeriodicUpdate();
  //     m_recordInformation = information;
  //     m_sendInformation = information;
  //   }
  // }
  m_checkChangeTimer.Schedule (m_checkChangeInterval + MicroSeconds (25 * m_uniformRandomVariable->GetInteger (0,1000)));
}

// 周期发送控制包
void
RoutingProtocol::SendPeriodicUpdate ()
{ 
  m_lastSendTime = Simulator::Now ().ToInteger(Time::S);

  Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
  Vector myPos = MM->GetPosition();
  Vector myVel = MM->GetVelocity();
  int16_t vx = (int16_t)myVel.x;
  int16_t vy = (int16_t)myVel.y;
  int16_t vz = (int16_t)myVel.z;
  uint16_t sign = SetRightVelocity(vx,vy,vz);

  // 在位置表中更新一下自己的位置信息，方便打印路由表分析
  RoutingTableEntry rt (/* x */(uint16_t)myPos.x, /* y */(uint16_t)myPos.y, /* z */(uint16_t)myPos.z, /* vx */vx, /* vy */vy, /* vz */vz,
                                    /* timestamp */Simulator::Now ().ToInteger(Time::S), /* adress */Ipv4Address::GetLoopback ());
  m_routingTable.Update(rt);

  MyprotocolHeader myprotocolHeader;
  myprotocolHeader.SetX((uint16_t)myPos.x);
  myprotocolHeader.SetY((uint16_t)myPos.y);
  myprotocolHeader.SetZ((uint16_t)myPos.z);
  myprotocolHeader.SetVx(abs(vx));
  myprotocolHeader.SetVy(abs(vy));
  myprotocolHeader.SetVz(abs(vz));
  myprotocolHeader.SetSign(sign);
  myprotocolHeader.SetTimestamp(m_lastSendTime);
  myprotocolHeader.SetMyadress(m_ipv4->GetAddress (1, 0).GetLocal ());

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (myprotocolHeader);

  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin (); j
       != m_socketAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;

      // ADD:在id-cache中添加这个控制包
      m_idCache.IsDuplicate (m_ipv4->GetAddress (1, 0).GetLocal (), myprotocolHeader.GetTimestamp());

      Ipv4Address destination;
      if (iface.GetMask () == Ipv4Mask::GetOnes ())
        {
          destination = Ipv4Address ("255.255.255.255");
        }
      else
        {
          destination = iface.GetBroadcast ();
        }
      socket->SendTo (packet, 0, InetSocketAddress (destination, MYPROTOCOL_PORT));
    }

  if(!m_enableAdaptiveUpdate){
    m_periodicUpdateTimer.Schedule (m_periodicUpdateInterval + MicroSeconds (25 * m_uniformRandomVariable->GetInteger (0,1000)));
  }
}

void
RoutingProtocol::SendPacketFromQueue (Ipv4Address dst, Ptr<Ipv4Route> route, DataHeader dataHeader)
{
  QueueEntry queueEntry;
  while (m_queue.Dequeue (dst, queueEntry))
    {
      Ptr<Packet> p = ConstCast<Packet> (queueEntry.GetPacket ());

      // 在queue中的数据包已经添加了udpheader的头
      PacketMetadata::ItemIterator i = p->BeginItem();
      PacketMetadata::Item item = i.Next ();
      TypeId id = item.tid;
      Icmpv4Header icmpv4Header;
      UdpHeader udpHeader;
      
      // icmpv4的包
      if(id == icmpv4Header.GetTypeId()){
        // 清除icmpv4的包头
        p->RemoveHeader(icmpv4Header);
      }else{
        // 清除udpheader的包头
        // 先清除前面多余的8个字节，就可以获得正确的header
        p->RemoveHeader(udpHeader);
      }

      DataHeader oldHeader;
      p->RemoveHeader(oldHeader);
      uint64_t uid = oldHeader.GetUid();
      dataHeader.SetUid(uid);
      p->AddHeader(dataHeader);
      // icmpv4的包
      if(id == icmpv4Header.GetTypeId()){
        // 清除icmpv4的包头
        p->AddHeader(icmpv4Header);
      }else{
        // 清除udpheader的包头
        // 先清除前面多余的8个字节，就可以获得正确的header
        p->AddHeader(udpHeader);
      }

      UnicastForwardCallback ucb = queueEntry.GetUnicastForwardCallback ();
      Ipv4Header header = queueEntry.GetIpv4Header ();
      ucb (route, p, header);
    }
}

uint16_t 
RoutingProtocol::SetRightVelocity(int16_t vx, int16_t vy, int16_t vz){
  uint16_t sign = 9;
  if(vx >= 0 && vy >= 0 && vz >= 0){
    sign = 0;
  }
  if(vx < 0 && vy >= 0 && vz >= 0){
    sign = 1;
  }
  if(vx >= 0 && vy < 0 && vz >= 0){
    sign = 2;
  }
  if(vx >= 0 && vy >= 0 && vz < 0){
    sign = 3;
  }
  if(vx < 0 && vy < 0 && vz >= 0){
    sign = 4;
  }
  if(vx < 0 && vy >= 0 && vz < 0){
    sign = 5;
  }
  if(vx >= 0 && vy < 0 && vz < 0){
    sign = 6;
  }
  if(vx < 0 && vy < 0 && vz < 0){
    sign = 7;
  }
  return sign;
}

Vector 
RoutingProtocol::GetRightVelocity(uint16_t vx, uint16_t vy, uint16_t vz, uint16_t sign){
  if(sign == 0){
    return Vector(vx,vy,vz);
  }
  if(sign == 1){
    return Vector(-vx,vy,vz);
  }
  if(sign == 2){
    return Vector(vx,-vy,vz);
  }
  if(sign == 3){
    return Vector(vx,vy,-vz);
  }
  if(sign == 4){
    return Vector(-vx,-vy,vz);
  }
  if(sign == 5){
    return Vector(-vx,vy,-vz);
  }
  if(sign == 6){
    return Vector(vx,-vy,-vz);
  }
  if(sign == 7){
    return Vector(-vx,-vy,-vz);
  }
  std::cout<<"sign is wrong!!!\n";
  return Vector(0,0,0);
}

//=====================================================================================================================

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  m_ipv4 = ipv4;
  // Create lo route. It is asserted that the only one interface up for now is loopback
  NS_ASSERT (m_ipv4->GetNInterfaces () == 1 && m_ipv4->GetAddress (0, 0).GetLocal () == Ipv4Address ("127.0.0.1"));
  m_lo = m_ipv4->GetNetDevice (0);
  NS_ASSERT (m_lo != 0);
  // Remember lo route
  // ADD：修改初始回环地址的路由表项
  RoutingTableEntry rt (
    /* x */0, /* y */0, /* z */0, /* vx */0, /* vy */0, /* vz */0,
    /* timestamp */0, /* adress */Ipv4Address::GetLoopback ());
  m_routingTable.AddRoute (rt); 
  Simulator::ScheduleNow (&RoutingProtocol::Start,this);
}

void
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{
  NS_LOG_FUNCTION (this << m_ipv4->GetAddress (i, 0).GetLocal ()
                        << " interface is up");
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  Ipv4InterfaceAddress iface = l3->GetAddress (i,0);
  if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
    {
      return;
    }
  // Create a socket to listen only on this interface
  Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),UdpSocketFactory::GetTypeId ());
  NS_ASSERT (socket != 0);
  socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvMyprotocol,this));
  socket->BindToNetDevice (l3->GetNetDevice (i));
  socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), MYPROTOCOL_PORT));
  socket->SetAllowBroadcast (true);
  socket->SetAttribute ("IpTtl",UintegerValue (1));
  m_socketAddresses.insert (std::make_pair (socket,iface));
  // Add local broadcast record to the routing table
  RoutingTableEntry rt (/* x */0, /* y */0, /* z */0, /* vx */0, /* vy */0, /* vz */0,
                                    /* timestamp */0, /* adress */iface.GetBroadcast ());
  m_routingTable.AddRoute (rt);
  if (m_mainAddress == Ipv4Address ())
    {
      m_mainAddress = iface.GetLocal ();
    }
  NS_ASSERT (m_mainAddress != Ipv4Address ());
}

void
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  Ptr<NetDevice> dev = l3->GetNetDevice (i);
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (m_ipv4->GetAddress (i,0));
  NS_ASSERT (socket);
  socket->Close ();
  m_socketAddresses.erase (socket);
  if (m_socketAddresses.empty ())
    {
      NS_LOG_LOGIC ("No myprotocol interfaces");
      m_routingTable.Clear ();
      return;
    }
}

void
RoutingProtocol::NotifyAddAddress (uint32_t i,
                                   Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION (this << " interface " << i << " address " << address);
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  if (!l3->IsUp (i))
    {
      return;
    }
  Ipv4InterfaceAddress iface = l3->GetAddress (i,0);
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (iface);
  if (!socket)
    {
      if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
        {
          return;
        }
      Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),UdpSocketFactory::GetTypeId ());
      NS_ASSERT (socket != 0);
      socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvMyprotocol,this));
      // Bind to any IP address so that broadcasts can be received
      socket->BindToNetDevice (l3->GetNetDevice (i));
      socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), MYPROTOCOL_PORT));
      socket->SetAllowBroadcast (true);
      m_socketAddresses.insert (std::make_pair (socket,iface));
      RoutingTableEntry rt (/* x */0, /* y */0, /* z */0, /* vx */0, /* vy */0, /* vz */0,
                                        /* timestamp */0, /* adress */iface.GetBroadcast ());
      m_routingTable.AddRoute (rt);
    }
}

void
RoutingProtocol::NotifyRemoveAddress (uint32_t i,
                                      Ipv4InterfaceAddress address)
{
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (address);
  if (socket)
    {
      m_socketAddresses.erase (socket);
      Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
      if (l3->GetNAddresses (i))
        {
          Ipv4InterfaceAddress iface = l3->GetAddress (i,0);
          // Create a socket to listen only on this interface
          Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),UdpSocketFactory::GetTypeId ());
          NS_ASSERT (socket != 0);
          socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvMyprotocol,this));
          // Bind to any IP address so that broadcasts can be received
          socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), MYPROTOCOL_PORT));
          socket->SetAllowBroadcast (true);
          m_socketAddresses.insert (std::make_pair (socket,iface));
        }
    }
}

Ptr<Socket>
RoutingProtocol::FindSocketWithInterfaceAddress (Ipv4InterfaceAddress addr) const
{
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin (); j
       != m_socketAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      if (iface == addr)
        {
          return socket;
        }
    }
  Ptr<Socket> socket;
  return socket;
}

void
RoutingProtocol::Send (Ptr<Ipv4Route> route,
                       Ptr<const Packet> packet,
                       const Ipv4Header & header)
{
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  NS_ASSERT (l3 != 0);
  Ptr<Packet> p = packet->Copy ();
  l3->Send (p,route->GetSource (),header.GetDestination (),header.GetProtocol (),route);
}

void
RoutingProtocol::Drop (Ptr<const Packet> packet,
                       const Ipv4Header & header,
                       Socket::SocketErrno err)
{
  NS_LOG_DEBUG (m_mainAddress << " drop packet " << packet->GetUid () << " to "
                              << header.GetDestination () << " from queue. Error " << err);
}
}
}
