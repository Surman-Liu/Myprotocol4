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

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MyprotocolRoutingProtocol");

namespace myprotocol {

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

/// UDP Port for myprotocol control traffic
const uint32_t RoutingProtocol::MYPROTOCOL_PORT = 269;

/// Tag used by myprotocol implementation
struct DeferredRouteOutputTag : public Tag
{
  /// Positive if output device is fixed in RouteOutput
  int32_t oif;

  /**
   * Constructor
   *
   * \param o outgoing interface (OIF)
   */
  DeferredRouteOutputTag (int32_t o = -1)
    : Tag (),
      oif (o)
  {
  }

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId
  GetTypeId ()
  {
    static TypeId tid = TypeId ("ns3::myprotocol::DeferredRouteOutputTag")
      .SetParent<Tag> ()
      .SetGroupName ("Myprotocol")
      .AddConstructor<DeferredRouteOutputTag> ()
    ;
    return tid;
  }

  TypeId
  GetInstanceTypeId () const
  {
    return GetTypeId ();
  }

  uint32_t
  GetSerializedSize () const
  {
    return sizeof(int32_t);
  }

  void
  Serialize (TagBuffer i) const
  {
    i.WriteU32 (oif);
  }

  void
  Deserialize (TagBuffer i)
  {
    oif = i.ReadU32 ();
  }

  void
  Print (std::ostream &os) const
  {
    os << "DeferredRouteOutputTag: output interface = " << oif;
  }
};

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
    .AddAttribute ("SettlingTime", "Minimum time an update is to be stored in adv table before sending out"
                   "in case of change in metric (in seconds)",
                   TimeValue (Seconds (5)),
                   MakeTimeAccessor (&RoutingProtocol::m_settlingTime),
                   MakeTimeChecker ())
    .AddAttribute ("EnableBuffering","Enables buffering of data packets if no route to destination is available",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RoutingProtocol::SetEnableBufferFlag,
                                        &RoutingProtocol::GetEnableBufferFlag),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableWST","Enables Weighted Settling Time for the updates before advertising",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RoutingProtocol::SetWSTFlag,
                                        &RoutingProtocol::GetWSTFlag),
                   MakeBooleanChecker ())
    .AddAttribute ("Holdtimes","Times the forwarding Interval to purge the route.",
                   UintegerValue (3),
                   MakeUintegerAccessor (&RoutingProtocol::Holdtimes),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("WeightedFactor","WeightedFactor for the settling time if Weighted Settling Time is enabled",
                   DoubleValue (0.875),
                   MakeDoubleAccessor (&RoutingProtocol::m_weightedFactor),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("EnableRouteAggregation","Enables Weighted Settling Time for the updates before advertising",
                   BooleanValue (false),
                   MakeBooleanAccessor (&RoutingProtocol::SetEnableRAFlag,
                                        &RoutingProtocol::GetEnableRAFlag),
                   MakeBooleanChecker ())
    .AddAttribute ("RouteAggregationTime","Time to aggregate updates before sending them out (in seconds)",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&RoutingProtocol::m_routeAggregationTime),
                   MakeTimeChecker ());
  return tid;
}

void
RoutingProtocol::SetEnableBufferFlag (bool f)
{
  EnableBuffering = f;
}
bool
RoutingProtocol::GetEnableBufferFlag () const
{
  return EnableBuffering;
}
void
RoutingProtocol::SetWSTFlag (bool f)
{
  EnableWST = f;
}
bool
RoutingProtocol::GetWSTFlag () const
{
  return EnableWST;
}
void
RoutingProtocol::SetEnableRAFlag (bool f)
{
  EnableRouteAggregation = f;
}
bool
RoutingProtocol::GetEnableRAFlag () const
{
  return EnableRouteAggregation;
}

int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

RoutingProtocol::RoutingProtocol ()
  : m_routingTable (),
    m_periodicUpdateTimer (Timer::CANCEL_ON_DESTROY)
{
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
  m_periodicUpdateTimer.SetFunction (&RoutingProtocol::SendPeriodicUpdate,this);
  m_periodicUpdateTimer.Schedule (MicroSeconds (m_uniformRandomVariable->GetInteger (0,1000)));
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
  NS_LOG_DEBUG ("Packet Size: " << p->GetSize ()
                                << ", Packet id: " << p->GetUid () << ", Destination address in Packet: " << dst);
  Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
  Vector myPos = MM->GetPosition();
  // ADD：发送自己的数据包和控制包
  // 如果目的地是广播的话，那说明这是一个控制包，直接广播即可。
  if(dst == m_ipv4->GetAddress (1, 0).GetBroadcast ()){
    route->SetDestination(m_ipv4->GetAddress (1, 0).GetBroadcast ());
    route->SetGateway(m_ipv4->GetAddress (1, 0).GetBroadcast ());
    route->SetSource(m_ipv4->GetAddress (1, 0).GetLocal ());
    route->SetOutputDevice(m_ipv4->GetNetDevice (1));
    return route;
  }
  else{
    // 目的地是单播，则是一个数据包，需要添加贪婪转发状态的包头，再进行贪婪转发。
    DataHeader dataHeader;
    Ipv4Address nexthop = m_routingTable.BestNeighbor(dst,myPos);
    if(nexthop != Ipv4Address::GetZero ()){
      RoutingTableEntry rt;
      m_routingTable.LookupRoute(dst,rt);
      dataHeader.SetDstPosx(rt.GetX());
      dataHeader.SetDstPosy(rt.GetY());
      dataHeader.SetDstPosz(rt.GetZ());
      dataHeader.SetUpdated(rt.GetTimestamp());
      dataHeader.SetRecPosx(0);
      dataHeader.SetRecPosy(0);
      dataHeader.SetRecPosz(0);
      dataHeader.SetInRec(0);
      dataHeader.SetLastPosx((uint16_t)myPos.x);
      dataHeader.SetLastPosy((uint16_t)myPos.y);
      dataHeader.SetLastPosz((uint16_t)myPos.z);
      p->AddHeader(dataHeader);
      route->SetDestination(header.GetDestination ());
      route->SetGateway(nexthop);
      if (header.GetSource () == Ipv4Address ("102.102.102.102"))
        {
          route->SetSource (m_ipv4->GetAddress (1, 0).GetLocal ());
        }
      else
        {
          route->SetSource (header.GetSource ());
        }
      route->SetOutputDevice (m_ipv4->GetNetDevice (1));

      return route;
    }
  }

  // todo
  // 没有找到合适的下一跳,符合恢复转发的条件（有目的地位置，有可转发邻居）
  // if(m_routingTable.MatchRecovery(dst,myPos)){
  //   // todo: 恢复模式获得下一跳
  //   return route;
  // }

  // 没有目的地的地址/没有邻居
  return LoopbackRoute (header,oif);
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
  NS_LOG_FUNCTION (m_mainAddress << " received packet " << p->GetUid ()
                                 << " from " << header.GetSource ()
                                 << " on interface " << idev->GetAddress ()
                                 << " to destination " << header.GetDestination ());
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
  if (EnableBuffering == true && idev == m_lo)
    {
      return false;
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

/// ADD：If route exists and valid, forward packet.
bool 
RoutingProtocol::Forwarding (Ptr<const Packet> packet, const Ipv4Header & header, UnicastForwardCallback ucb, ErrorCallback ecb){
  Ptr<Packet> p = packet->Copy ();
  Ipv4Address dst = header.GetDestination ();
  Ipv4Address origin = header.GetSource ();

  Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
  Vector myPos = MM->GetPosition();

  uint16_t updated = 0;
  Vector DstPosition;
  Vector RecPosition;
  uint16_t inRec = 0;
  Vector previousHop;

  DataHeader dataHeader;
  p->RemoveHeader(dataHeader);
  DstPosition.x = dataHeader.GetDstPosx ();
  DstPosition.y = dataHeader.GetDstPosy ();
  DstPosition.z = dataHeader.GetDstPosz ();
  updated = dataHeader.GetUpdated ();
  RecPosition.x = dataHeader.GetRecPosx ();
  RecPosition.y = dataHeader.GetRecPosy ();
  RecPosition.z = dataHeader.GetRecPosz ();
  inRec = dataHeader.GetInRec ();
  previousHop.x = dataHeader.GetLastPosx ();
  previousHop.y = dataHeader.GetLastPosy ();
  previousHop.z = dataHeader.GetLastPosz ();

  Ipv4Address nextHop;
  std::map<Ipv4Address, RoutingTableEntry> neighborTable;
  m_routingTable.LookupNeighbor(neighborTable, myPos);
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = neighborTable.find (dst);
  // 不论是否使用贪婪，如果目的地就是我的邻居，可以认为变成使用贪婪，直接ucb转发。
  if(i != neighborTable.end ())
    {
      nextHop = dst;
      DataHeader dthdr ((uint16_t)DstPosition.x, (uint16_t)DstPosition.y, (uint16_t)DstPosition.y, updated, 0, 0, 0, 0, 
                        (uint16_t)myPos.x, (uint16_t)myPos.y, (uint16_t)myPos.z);
      p->AddHeader (dthdr);
      Ptr<Ipv4Route> route = Create<Ipv4Route> ();
      route->SetDestination (dst);
      route->SetGateway (dst);
      route->SetOutputDevice (m_ipv4->GetNetDevice (1));
      route->SetSource (header.GetSource ());
      ucb (route, p, header);
      return true;
    }
  
  // 如果目的地不是自己的邻居，也不是自己，则需要切换到正确的转发模式进行转发

  // 收到恢复状态发过来的包，如果距离比RecPosition更近，则切换为贪婪状态。
  if(inRec == 1 && CalculateDistance (myPos, DstPosition) < CalculateDistance (RecPosition, DstPosition)){
    inRec = 0;
    dataHeader.SetInRec(0);
    NS_LOG_LOGIC ("No longer in Recovery to " << dst << " in " << myPos);
  }
  // 如果数据包本身就是恢复模式，并且该节点到目的地的距离比进入恢复模式到目的地的距离更远，则继续恢复模式
  if(inRec == 1){
    dataHeader.SetLastPosx ((uint16_t)myPos.x); //when entering Recovery, the first edge is the Dst
    dataHeader.SetLastPosy ((uint16_t)myPos.y); 
    dataHeader.SetLastPosz ((uint16_t)myPos.z);
    p->AddHeader (dataHeader);
    // todo:恢复模式
    // RecoveryMode (dst, p, ucb, header);
    // return true;
    return false;
  }
  if(inRec == 0){     //贪婪模式
    // 使用贪婪来寻找邻居表中离目的地最近的下一跳。
    nextHop = m_routingTable.BestNeighbor (dst, myPos);
    if (nextHop != Ipv4Address::GetZero ())
    {
      DataHeader dthdr (DstPosition.x, DstPosition.y, DstPosition.z, updated, 0, 0, 0, 0, 
                                (uint16_t)myPos.x, (uint16_t)myPos.y, (uint16_t)myPos.z);
      p->AddHeader (dthdr);          
      Ptr<Ipv4Route> route = Create<Ipv4Route> ();
      route->SetDestination (dst);
      route->SetSource (header.GetSource ());
      route->SetGateway (nextHop);
      route->SetOutputDevice (m_ipv4->GetNetDevice (1));
      NS_LOG_DEBUG ("Exist route to " << route->GetDestination () << " from interface " << route->GetOutputDevice ());                
      NS_LOG_LOGIC (route->GetOutputDevice () << " forwarding to " << dst << " from " << origin << " through " << route->GetGateway () << " packet " << p->GetUid ());    
      ucb (route, p, header);
      return true;
    }else{       // 如果贪婪转发没有找到合适的下一跳
      inRec = 1;
      dataHeader.SetInRec(1);
      dataHeader.SetRecPosx ((uint16_t)myPos.x);
      dataHeader.SetRecPosy ((uint16_t)myPos.y); 
      dataHeader.SetRecPosz ((uint16_t)myPos.z);
      dataHeader.SetLastPosx ((uint16_t)DstPosition.x); //when entering Recovery, the first edge is the Dst
      dataHeader.SetLastPosy ((uint16_t)DstPosition.y); 
      dataHeader.SetLastPosz ((uint16_t)DstPosition.z); 
      p->AddHeader (dataHeader);
      // todo:恢复模式
      // RecoveryMode (dst, p, ucb, header);
      return false;
    }
  }

  return false;
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
  // rt->SetSource (hdr.GetSource ());
  //
  // Source address selection here is tricky.  The loopback route is
  // returned when myprotocol does not have a route; this causes the packet
  // to be looped back and handled (cached) in RouteInput() method
  // while a route is found. However, connection-oriented protocols
  // like TCP need to create an endpoint four-tuple (src, src port,
  // dst, dst port) and create a pseudo-header for checksumming.  So,
  // myprotocol needs to guess correctly what the eventual source address
  // will be.
  //
  // For single interface, single address nodes, this is not a problem.
  // When there are possibly multiple outgoing interfaces, the policy
  // implemented here is to pick the first available myprotocol interface.
  // If RouteOutput() caller specified an outgoing interface, that
  // further constrains the selection of source address
  //
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
  NS_ASSERT_MSG (rt->GetSource () != Ipv4Address (), "Valid myprotocol source address not found");
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
  // std::cout<<"packet size = "<<packet->GetSize()<<"\n";

  MyprotocolHeader myprotocolHeader;
  packet->RemoveHeader (myprotocolHeader);
  Ipv4Address source = myprotocolHeader.GetMyadress();         //是哪个节点的位置信息
  // 获取带符号的速度信息
  Vector velocity = GetRightVelocity(myprotocolHeader.GetVx(), myprotocolHeader.GetVy(), myprotocolHeader.GetVz(), myprotocolHeader.GetSign());
  RoutingTableEntry rt;
  // 注意，位置表中找的是source的地址，更新的是source的信息
  // 是从std::map<Ipv4Address, RoutingTableEntry>中找Ipv4Address
  if(!m_routingTable.LookupRoute(source,rt)){       //如果位置表中没有该表项，则添加
    RoutingTableEntry newEntry (
      myprotocolHeader.GetX(),
      myprotocolHeader.GetY(),
      myprotocolHeader.GetZ(),
      velocity.x,
      velocity.y,
      velocity.z,
      myprotocolHeader.GetTimestamp(),
      myprotocolHeader.GetMyadress());
      m_routingTable.AddRoute(newEntry);
  }else{
    if(rt.GetTimestamp() < myprotocolHeader.GetTimestamp()){         //如果位置表中的表项比控制包中信息更新，则更新控制包信息
      rt.SetX(myprotocolHeader.GetX());
      rt.SetY(myprotocolHeader.GetY());
      rt.SetZ(myprotocolHeader.GetZ());
      rt.SetVx(velocity.x);
      rt.SetVy(velocity.y);
      rt.SetVz(velocity.z);
      rt.SetTimestamp(myprotocolHeader.GetTimestamp());
      rt.SetAdress(myprotocolHeader.GetMyadress());
      m_routingTable.Update(rt);
    }
  }
}

// 周期发送控制包
void
RoutingProtocol::SendPeriodicUpdate ()
{
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
  myprotocolHeader.SetTimestamp(Simulator::Now ().ToInteger(Time::S));
  myprotocolHeader.SetMyadress(m_ipv4->GetAddress (1, 0).GetLocal ());
  // std::cout<<"send timestamp = "<<Simulator::Now ().ToInteger(Time::S)<<"; myadress = "<<m_ipv4->GetAddress (1, 0).GetLocal ()<<"\n";

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (myprotocolHeader);

  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin (); j
       != m_socketAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      // question：这个send函数在这里有什么用？？？
      socket->Send (packet);
      // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
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
      NS_LOG_FUNCTION ("PeriodicUpdate Packet UID is : " << packet->GetUid ());
    }
  m_periodicUpdateTimer.Schedule (m_periodicUpdateInterval + MicroSeconds (25 * m_uniformRandomVariable->GetInteger (0,1000)));
}

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
  // rt.SetFlag (INVALID);
  // rt.SetEntriesChanged (false);
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

//=====================================================================================================================

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
