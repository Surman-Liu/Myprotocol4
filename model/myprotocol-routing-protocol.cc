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
    .AddAttribute ("MaxQueueLen", "Maximum number of packets that we allow a routing protocol to buffer.",
                   UintegerValue (500 /*assuming maximum nodes in simulation is 100*/),
                   MakeUintegerAccessor (&RoutingProtocol::m_maxQueueLen),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("MaxQueuedPacketsPerDst", "Maximum number of packets that we allow per destination to buffer.",
                   UintegerValue (5),
                   MakeUintegerAccessor (&RoutingProtocol::m_maxQueuedPacketsPerDst),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("MaxQueueTime","Maximum time packets can be queued (in seconds)",
                   TimeValue (Seconds (30)),
                   MakeTimeAccessor (&RoutingProtocol::m_maxQueueTime),
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
    m_advRoutingTable (),
    m_queue (),
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
  m_queue.SetMaxPacketsPerDst (m_maxQueuedPacketsPerDst);
  m_queue.SetMaxQueueLen (m_maxQueueLen);
  m_queue.SetQueueTimeout (m_maxQueueTime);
  m_routingTable.Setholddowntime (Time (Holdtimes * m_periodicUpdateInterval));
  m_advRoutingTable.Setholddowntime (Time (Holdtimes * m_periodicUpdateInterval));
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
  std::map<Ipv4Address, RoutingTableEntry> removedAddresses;
  sockerr = Socket::ERROR_NOTERROR;
  Ptr<Ipv4Route> route;
  Ipv4Address dst = header.GetDestination ();
  NS_LOG_DEBUG ("Packet Size: " << p->GetSize ()
                                << ", Packet id: " << p->GetUid () << ", Destination address in Packet: " << dst);
  RoutingTableEntry rt;
  m_routingTable.Purge (removedAddresses);
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator rmItr = removedAddresses.begin ();
       rmItr != removedAddresses.end (); ++rmItr)
    {
      rmItr->second.SetEntriesChanged (true);
      rmItr->second.SetSeqNo (rmItr->second.GetSeqNo () + 1);
      m_advRoutingTable.AddRoute (rmItr->second);
    }
  if (!removedAddresses.empty ())
    {
      Simulator::Schedule (MicroSeconds (m_uniformRandomVariable->GetInteger (0,1000)),&RoutingProtocol::SendTriggeredUpdate,this);
    }
  if (m_routingTable.LookupRoute (dst,rt))
    {
      if (EnableBuffering)
        {
          LookForQueuedPackets ();
        }
      if (rt.GetHop () == 1)
        {
          route = rt.GetRoute ();
          NS_ASSERT (route != 0);
          NS_LOG_DEBUG ("A route exists from " << route->GetSource ()
                                               << " to neighboring destination "
                                               << route->GetDestination ());
          if (oif != 0 && route->GetOutputDevice () != oif)
            {
              NS_LOG_DEBUG ("Output device doesn't match. Dropped.");
              sockerr = Socket::ERROR_NOROUTETOHOST;
              return Ptr<Ipv4Route> ();
            }
          return route;
        }
      else
        {
          RoutingTableEntry newrt;
          if (m_routingTable.LookupRoute (rt.GetNextHop (),newrt))
            {
              route = newrt.GetRoute ();
              NS_ASSERT (route != 0);
              NS_LOG_DEBUG ("A route exists from " << route->GetSource ()
                                                   << " to destination " << dst << " via "
                                                   << rt.GetNextHop ());
              if (oif != 0 && route->GetOutputDevice () != oif)
                {
                  NS_LOG_DEBUG ("Output device doesn't match. Dropped.");
                  sockerr = Socket::ERROR_NOROUTETOHOST;
                  return Ptr<Ipv4Route> ();
                }
              return route;
            }
        }
    }

  if (EnableBuffering)
    {
      uint32_t iif = (oif ? m_ipv4->GetInterfaceForDevice (oif) : -1);
      DeferredRouteOutputTag tag (iif);
      if (!p->PeekPacketTag (tag))
        {
          p->AddPacketTag (tag);
        }
    }
  return LoopbackRoute (header,oif);
}

void
RoutingProtocol::DeferredRouteOutput (Ptr<const Packet> p,
                                      const Ipv4Header & header,
                                      UnicastForwardCallback ucb,
                                      ErrorCallback ecb)
{
  NS_LOG_FUNCTION (this << p << header);
  NS_ASSERT (p != 0 && p != Ptr<Packet> ());
  QueueEntry newEntry (p,header,ucb,ecb);
  bool result = m_queue.Enqueue (newEntry);
  if (result)
    {
      NS_LOG_DEBUG ("Added packet " << p->GetUid () << " to queue.");
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
  if (dst.IsMulticast ())
    {
      return false;
    }

  // Deferred route request
  if (EnableBuffering == true && idev == m_lo)
    {
      DeferredRouteOutputTag tag;
      if (p->PeekPacketTag (tag))
        {
          DeferredRouteOutput (p,header,ucb,ecb);
          return true;
        }
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
  // LOCAL DELIVARY TO myprotocol INTERFACES
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
                  if (m_routingTable.LookupRoute (dst,toBroadcast,true))
                    {
                      Ptr<Ipv4Route> route = toBroadcast.GetRoute ();
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

  if (m_ipv4->IsDestinationAddress (dst, iif))
    {
      if (lcb.IsNull () == false)
        {
          NS_LOG_LOGIC ("Unicast local delivery to " << dst);
          lcb (p, header, iif);
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

  RoutingTableEntry toDst;
  if (m_routingTable.LookupRoute (dst,toDst))
    {
      RoutingTableEntry ne;
      if (m_routingTable.LookupRoute (toDst.GetNextHop (),ne))
        {
          Ptr<Ipv4Route> route = ne.GetRoute ();
          NS_LOG_LOGIC (m_mainAddress << " is forwarding packet " << p->GetUid ()
                                      << " to " << dst
                                      << " from " << header.GetSource ()
                                      << " via nexthop neighbor " << toDst.GetNextHop ());
          ucb (route,p,header);
          return true;
        }
    }
  NS_LOG_LOGIC ("Drop packet " << p->GetUid ()
                               << " as there is no route to forward it.");
  return false;
}

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

void
RoutingProtocol::RecvMyprotocol (Ptr<Socket> socket)
{
  Address sourceAddress;
  Ptr<Packet> advpacket = Create<Packet> ();
  Ptr<Packet> packet = socket->RecvFrom (sourceAddress);
  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address sender = inetSourceAddr.GetIpv4 ();
  Ipv4Address receiver = m_socketAddresses[socket].GetLocal ();
  Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver));
  uint32_t packetSize = packet->GetSize ();
  NS_LOG_FUNCTION (m_mainAddress << " received myprotocol packet of size: " << packetSize
                                 << " and packet id: " << packet->GetUid ());
  uint32_t count = 0;
  // ADD: 循环减去的数量要和header长度相同
  for (; packetSize > 0; packetSize = packetSize - 28)
    {
      count = 0;
      MyprotocolHeader myprotocolHeader, tempMyprotocolHeader;
      packet->RemoveHeader (myprotocolHeader);
      NS_LOG_DEBUG ("Processing new update for " << myprotocolHeader.GetDst ());
      /*Verifying if the packets sent by me were returned back to me. If yes, discarding them!*/
      for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin (); j
           != m_socketAddresses.end (); ++j)
        {
          Ipv4InterfaceAddress interface = j->second;
          if (myprotocolHeader.GetDst () == interface.GetLocal ())
            {
              if (myprotocolHeader.GetDstSeqno () % 2 == 1)
                {
                  NS_LOG_DEBUG ("Sent myprotocol update back to the same Destination, "
                                "with infinite metric. Time left to send fwd update: "
                                << m_periodicUpdateTimer.GetDelayLeft ());
                  count++;
                }
              else
                {
                  NS_LOG_DEBUG ("Received update for my address. Discarding this.");
                  count++;
                }
            }
        }
      if (count > 0)
        {
          continue;
        }
      NS_LOG_DEBUG ("Received a myprotocol packet from "
                    << sender << " to " << receiver << ". Details are: Destination: " << myprotocolHeader.GetDst () << ", Seq No: "
                    << myprotocolHeader.GetDstSeqno () << ", HopCount: " << myprotocolHeader.GetHopCount ());

      // ADD:判断该控制包包头的速度符号
      Vector velocity = GetRightVelocity(myprotocolHeader.GetVx(), myprotocolHeader.GetVy(), myprotocolHeader.GetVz(), myprotocolHeader.GetSign());
      // ADD:测试和调试信息打印
      // std::cout<<"x = "<<myprotocolHeader.GetX()<<",y = "<<myprotocolHeader.GetY()<<",z = "<<myprotocolHeader.GetZ()<<"\n";
      // std::cout<<"vx = "<<myprotocolHeader.GetVx()<<",vy = "<<myprotocolHeader.GetVy()<<",vz = "<<myprotocolHeader.GetVz()<<",sign = "<<myprotocolHeader.GetSign()<<"\n";
      // std::cout<<"right velocity = "<<velocity<<"\n";
      RoutingTableEntry fwdTableEntry, advTableEntry;
      EventId event;
      bool permanentTableVerifier = m_routingTable.LookupRoute (myprotocolHeader.GetDst (),fwdTableEntry);
      if (permanentTableVerifier == false)
        {
          // ADD:更新路由表的位置、速度、时间戳信息
          if (myprotocolHeader.GetDstSeqno () % 2 != 1)
            {
              NS_LOG_DEBUG ("Received New Route!");
              RoutingTableEntry newEntry (
                /*device=*/ dev, /*dst=*/
                myprotocolHeader.GetDst (), /*seqno=*/
                myprotocolHeader.GetDstSeqno (),
                /*iface=*/ m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0),
                /*hops=*/ myprotocolHeader.GetHopCount (), /*next hop=*/
                sender, /*lifetime=*/
                Simulator::Now (), /*settlingTime*/
                m_settlingTime, /*entries changed*/
                true,
                myprotocolHeader.GetX(),
                myprotocolHeader.GetY(),
                myprotocolHeader.GetZ(),
                velocity.x,
                velocity.y,
                velocity.z,
                myprotocolHeader.GetTimestamp());
              newEntry.SetFlag (VALID);
              m_routingTable.AddRoute (newEntry);
              NS_LOG_DEBUG ("New Route added to both tables");
              m_advRoutingTable.AddRoute (newEntry);
            }
          else
            {
              // received update not present in main routing table and also with infinite metric
              NS_LOG_DEBUG ("Discarding this update as this route is not present in "
                            "main routing table and received with infinite metric");
            }
        }
      else
        {
          if (!m_advRoutingTable.LookupRoute (myprotocolHeader.GetDst (),advTableEntry))
            {
              RoutingTableEntry tr;
              std::map<Ipv4Address, RoutingTableEntry> allRoutes;
              m_advRoutingTable.GetListOfAllRoutes (allRoutes);
              for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = allRoutes.begin (); i != allRoutes.end (); ++i)
                {
                  NS_LOG_DEBUG ("ADV table routes are:" << i->second.GetDestination ());
                }
              // present in fwd table and not in advtable
              m_advRoutingTable.AddRoute (fwdTableEntry);
              m_advRoutingTable.LookupRoute (myprotocolHeader.GetDst (),advTableEntry);
            }
          if (myprotocolHeader.GetDstSeqno () % 2 != 1)
            {
              if (myprotocolHeader.GetDstSeqno () > advTableEntry.GetSeqNo ())
                {
                  // Received update with better seq number. Clear any old events that are running
                  if (m_advRoutingTable.ForceDeleteIpv4Event (myprotocolHeader.GetDst ()))
                    {
                      NS_LOG_DEBUG ("Canceling the timer to update route with better seq number");
                    }
                  // if its a changed metric *nomatter* where the update came from, wait  for WST
                  if (myprotocolHeader.GetHopCount () != advTableEntry.GetHop ())
                    {
                      advTableEntry.SetSeqNo (myprotocolHeader.GetDstSeqno ());
                      advTableEntry.SetLifeTime (Simulator::Now ());
                      advTableEntry.SetFlag (VALID);
                      advTableEntry.SetEntriesChanged (true);
                      advTableEntry.SetNextHop (sender);
                      advTableEntry.SetHop (myprotocolHeader.GetHopCount ());
                      NS_LOG_DEBUG ("Received update with better sequence number and changed metric.Waiting for WST");
                      Time tempSettlingtime = GetSettlingTime (myprotocolHeader.GetDst ());
                      advTableEntry.SetSettlingTime (tempSettlingtime);
                      NS_LOG_DEBUG ("Added Settling Time:" << tempSettlingtime.GetSeconds ()
                                                           << "s as there is no event running for this route");
                      // ADD:更新路由表
                      advTableEntry.SetX(myprotocolHeader.GetX());
                      advTableEntry.SetY(myprotocolHeader.GetY());
                      advTableEntry.SetZ(myprotocolHeader.GetZ());
                      advTableEntry.SetVx(velocity.x);
                      advTableEntry.SetVy(velocity.y);
                      advTableEntry.SetVz(velocity.z);
                      advTableEntry.SetTimestamp(myprotocolHeader.GetTimestamp());
                      event = Simulator::Schedule (tempSettlingtime,&RoutingProtocol::SendTriggeredUpdate,this);
                      m_advRoutingTable.AddIpv4Event (myprotocolHeader.GetDst (),event);
                      NS_LOG_DEBUG ("EventCreated EventUID: " << event.GetUid ());
                      // if received changed metric, use it but adv it only after wst
                      m_routingTable.Update (advTableEntry);
                      m_advRoutingTable.Update (advTableEntry);
                    }
                  else
                    {
                      // Received update with better seq number and same metric.
                      advTableEntry.SetSeqNo (myprotocolHeader.GetDstSeqno ());
                      advTableEntry.SetLifeTime (Simulator::Now ());
                      advTableEntry.SetFlag (VALID);
                      advTableEntry.SetEntriesChanged (true);
                      advTableEntry.SetNextHop (sender);
                      advTableEntry.SetHop (myprotocolHeader.GetHopCount ());
                      // ADD：更新路由表
                      advTableEntry.SetX(myprotocolHeader.GetX());
                      advTableEntry.SetY(myprotocolHeader.GetY());
                      advTableEntry.SetZ(myprotocolHeader.GetZ());
                      advTableEntry.SetVx(velocity.x);
                      advTableEntry.SetVy(velocity.y);
                      advTableEntry.SetVz(velocity.z);
                      advTableEntry.SetTimestamp(myprotocolHeader.GetTimestamp());
                      m_advRoutingTable.Update (advTableEntry);
                      NS_LOG_DEBUG ("Route with better sequence number and same metric received. Advertised without WST");
                    }
                }
              else if (myprotocolHeader.GetDstSeqno () == advTableEntry.GetSeqNo ())
                {
                  if (myprotocolHeader.GetHopCount () < advTableEntry.GetHop ())
                    {
                      /*Received update with same seq number and better hop count.
                       * As the metric is changed, we will have to wait for WST before sending out this update.
                       */
                      NS_LOG_DEBUG ("Canceling any existing timer to update route with same sequence number "
                                    "and better hop count");
                      m_advRoutingTable.ForceDeleteIpv4Event (myprotocolHeader.GetDst ());
                      advTableEntry.SetSeqNo (myprotocolHeader.GetDstSeqno ());
                      advTableEntry.SetLifeTime (Simulator::Now ());
                      advTableEntry.SetFlag (VALID);
                      advTableEntry.SetEntriesChanged (true);
                      advTableEntry.SetNextHop (sender);
                      advTableEntry.SetHop (myprotocolHeader.GetHopCount ());
                      Time tempSettlingtime = GetSettlingTime (myprotocolHeader.GetDst ());
                      advTableEntry.SetSettlingTime (tempSettlingtime);
                      NS_LOG_DEBUG ("Added Settling Time," << tempSettlingtime.GetSeconds ()
                                                           << " as there is no current event running for this route");
                      // ADD：更新路由表
                      advTableEntry.SetX(myprotocolHeader.GetX());
                      advTableEntry.SetY(myprotocolHeader.GetY());
                      advTableEntry.SetZ(myprotocolHeader.GetZ());
                      advTableEntry.SetVx(velocity.x);
                      advTableEntry.SetVy(velocity.y);
                      advTableEntry.SetVz(velocity.z);
                      advTableEntry.SetTimestamp(myprotocolHeader.GetTimestamp());
                      event = Simulator::Schedule (tempSettlingtime,&RoutingProtocol::SendTriggeredUpdate,this);
                      m_advRoutingTable.AddIpv4Event (myprotocolHeader.GetDst (),event);
                      NS_LOG_DEBUG ("EventCreated EventUID: " << event.GetUid ());
                      // if received changed metric, use it but adv it only after wst
                      m_routingTable.Update (advTableEntry);
                      m_advRoutingTable.Update (advTableEntry);
                    }
                  else
                    {
                      /*Received update with same seq number but with same or greater hop count.
                       * Discard that update.
                       */
                      if (!m_advRoutingTable.AnyRunningEvent (myprotocolHeader.GetDst ()))
                        {
                          /*update the timer only if nexthop address matches thus discarding
                           * updates to that destination from other nodes.
                           */
                          if (advTableEntry.GetNextHop () == sender)
                            {
                              advTableEntry.SetLifeTime (Simulator::Now ());
                              m_routingTable.Update (advTableEntry);
                            }
                          m_advRoutingTable.DeleteRoute (
                            myprotocolHeader.GetDst ());
                        }
                      NS_LOG_DEBUG ("Received update with same seq number and "
                                    "same/worst metric for, " << myprotocolHeader.GetDst () << ". Discarding the update.");
                    }
                }
              else
                {
                  // Received update with an old sequence number. Discard the update
                  if (!m_advRoutingTable.AnyRunningEvent (myprotocolHeader.GetDst ()))
                    {
                      m_advRoutingTable.DeleteRoute (myprotocolHeader.GetDst ());
                    }
                  NS_LOG_DEBUG (myprotocolHeader.GetDst () << " : Received update with old seq number. Discarding the update.");
                }
            }
          else
            {
              NS_LOG_DEBUG ("Route with infinite metric received for "
                            << myprotocolHeader.GetDst () << " from " << sender);
              // Delete route only if update was received from my nexthop neighbor
              if (sender == advTableEntry.GetNextHop ())
                {
                  NS_LOG_DEBUG ("Triggering an update for this unreachable route:");
                  std::map<Ipv4Address, RoutingTableEntry> dstsWithNextHopSrc;
                  m_routingTable.GetListOfDestinationWithNextHop (myprotocolHeader.GetDst (),dstsWithNextHopSrc);
                  m_routingTable.DeleteRoute (myprotocolHeader.GetDst ());
                  advTableEntry.SetSeqNo (myprotocolHeader.GetDstSeqno ());
                  advTableEntry.SetEntriesChanged (true);
                  m_advRoutingTable.Update (advTableEntry);
                  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = dstsWithNextHopSrc.begin (); i
                       != dstsWithNextHopSrc.end (); ++i)
                    {
                      i->second.SetSeqNo (i->second.GetSeqNo () + 1);
                      i->second.SetEntriesChanged (true);
                      m_advRoutingTable.AddRoute (i->second);
                      m_routingTable.DeleteRoute (i->second.GetDestination ());
                    }
                }
              else
                {
                  if (!m_advRoutingTable.AnyRunningEvent (myprotocolHeader.GetDst ()))
                    {
                      m_advRoutingTable.DeleteRoute (myprotocolHeader.GetDst ());
                    }
                  NS_LOG_DEBUG (myprotocolHeader.GetDst () <<
                                " : Discard this link break update as it was received from a different neighbor "
                                "and I can reach the destination");
                }
            }
        }
    }
  std::map<Ipv4Address, RoutingTableEntry> allRoutes;
  m_advRoutingTable.GetListOfAllRoutes (allRoutes);
  if (EnableRouteAggregation && allRoutes.size () > 0)
    {
      Simulator::Schedule (m_routeAggregationTime,&RoutingProtocol::SendTriggeredUpdate,this);
    }
  else
    {
      Simulator::Schedule (MicroSeconds (m_uniformRandomVariable->GetInteger (0,1000)),&RoutingProtocol::SendTriggeredUpdate,this);
    }
}

// 更新m_advRoutingTable中的数据，只有在settlingtime之后才会广播
void
RoutingProtocol::SendTriggeredUpdate ()
{
  NS_LOG_FUNCTION (m_mainAddress << " is sending a triggered update");
  std::map<Ipv4Address, RoutingTableEntry> allRoutes;
  m_advRoutingTable.GetListOfAllRoutes (allRoutes);
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin (); j
       != m_socketAddresses.end (); ++j)
    {
      MyprotocolHeader myprotocolHeader;
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      Ptr<Packet> packet = Create<Packet> ();
      for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = allRoutes.begin (); i != allRoutes.end (); ++i)
        {
          NS_LOG_LOGIC ("Destination: " << i->second.GetDestination ()
                                        << " SeqNo:" << i->second.GetSeqNo () << " HopCount:"
                                        << i->second.GetHop () + 1);
          RoutingTableEntry temp = i->second;
          if ((i->second.GetEntriesChanged () == true) && (!m_advRoutingTable.AnyRunningEvent (temp.GetDestination ())))
            {
              myprotocolHeader.SetDst (i->second.GetDestination ());
              myprotocolHeader.SetDstSeqno (i->second.GetSeqNo ());
              myprotocolHeader.SetHopCount (i->second.GetHop () + 1);
              // ADD:将自己的路由表信息转换成header包头的信息。
              myprotocolHeader.SetX(i->second.GetX());
              myprotocolHeader.SetY(i->second.GetY());
              myprotocolHeader.SetZ(i->second.GetZ());
              myprotocolHeader.SetVx(abs(i->second.GetVx()));
              myprotocolHeader.SetVy(abs(i->second.GetVy()));
              myprotocolHeader.SetVz(abs(i->second.GetVz()));
              uint16_t sign = SetRightVelocity(i->second.GetVx(),i->second.GetVy(),i->second.GetVz());
              myprotocolHeader.SetSign(sign);
              myprotocolHeader.SetTimestamp(i->second.GetTimestamp());

              temp.SetFlag (VALID);
              temp.SetEntriesChanged (false);
              m_advRoutingTable.DeleteIpv4Event (temp.GetDestination ());
              if (!(temp.GetSeqNo () % 2))
                {
                  m_routingTable.Update (temp);
                }
              packet->AddHeader (myprotocolHeader);
              m_advRoutingTable.DeleteRoute (temp.GetDestination ());
              NS_LOG_DEBUG ("Deleted this route from the advertised table");
            }
          else
            {
              EventId event = m_advRoutingTable.GetEventId (temp.GetDestination ());
              NS_ASSERT (event.GetUid () != 0);
              NS_LOG_DEBUG ("EventID " << event.GetUid () << " associated with "
                                       << temp.GetDestination () << " has not expired, waiting in adv table");
            }
        }
      // ADD：这里的数是header的长度，因为packet大于header长度的话表示在数据宝里添加了路由表的内容，则再加上节点本身的信息，即可广播更新。
      if (packet->GetSize () >= 28)
        {
          RoutingTableEntry temp2;
          m_routingTable.LookupRoute (m_ipv4->GetAddress (1, 0).GetBroadcast (), temp2);
          myprotocolHeader.SetDst (m_ipv4->GetAddress (1, 0).GetLocal ());
          myprotocolHeader.SetDstSeqno (temp2.GetSeqNo ());
          myprotocolHeader.SetHopCount (temp2.GetHop () + 1);

          // ADD:将自己的位置信息加入到header中,这里有一个强制转换
          Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
          Vector myPos = MM->GetPosition();
          myprotocolHeader.SetX((uint16_t)myPos.x);
          myprotocolHeader.SetY((uint16_t)myPos.y);
          myprotocolHeader.SetZ((uint16_t)myPos.z);
          Vector myVel = MM->GetVelocity();
          int16_t vx = (int16_t)myVel.x;
          int16_t vy = (int16_t)myVel.y;
          int16_t vz = (int16_t)myVel.z;
          uint16_t sign = SetRightVelocity(vx,vy,vz);
          myprotocolHeader.SetVx(abs(vx));
          myprotocolHeader.SetVy(abs(vy));
          myprotocolHeader.SetVz(abs(vz));
          myprotocolHeader.SetSign(sign);
          myprotocolHeader.SetTimestamp(Simulator::Now ().ToInteger(Time::S));

          NS_LOG_DEBUG ("Adding my update as well to the packet");
          packet->AddHeader (myprotocolHeader);
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
          NS_LOG_FUNCTION ("Sent Triggered Update from "
                           << myprotocolHeader.GetDst ()
                           << " with packet id : " << packet->GetUid () << " and packet Size: " << packet->GetSize ());
        }
      else
        {
          NS_LOG_FUNCTION ("Update not sent as there are no updates to be triggered");
        }
    }
}

void
RoutingProtocol::SendPeriodicUpdate ()
{
  std::map<Ipv4Address, RoutingTableEntry> removedAddresses, allRoutes;
  m_routingTable.Purge (removedAddresses);
  MergeTriggerPeriodicUpdates ();
  m_routingTable.GetListOfAllRoutes (allRoutes);
  if (allRoutes.empty ())
    {
      return;
    }
  NS_LOG_FUNCTION (m_mainAddress << " is sending out its periodic update");
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin (); j
       != m_socketAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      Ptr<Packet> packet = Create<Packet> ();
      for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = allRoutes.begin (); i != allRoutes.end (); ++i)
        {
          MyprotocolHeader myprotocolHeader;
          // 发送自己的信息
          if (i->second.GetHop () == 0)
            {
              RoutingTableEntry ownEntry;
              myprotocolHeader.SetDst (m_ipv4->GetAddress (1,0).GetLocal ());
              myprotocolHeader.SetDstSeqno (i->second.GetSeqNo () + 2);
              myprotocolHeader.SetHopCount (i->second.GetHop () + 1);
              
              // ADD:将自己的位置信息加入到header中,这里有一个强制转换
              Ptr<MobilityModel> MM = m_ipv4->GetObject<MobilityModel> ();
              Vector myPos = MM->GetPosition();
              myprotocolHeader.SetX((uint16_t)myPos.x);
              myprotocolHeader.SetY((uint16_t)myPos.y);
              myprotocolHeader.SetZ((uint16_t)myPos.z);
              Vector myVel = MM->GetVelocity();
              int16_t vx = (int16_t)myVel.x;
              int16_t vy = (int16_t)myVel.y;
              int16_t vz = (int16_t)myVel.z;
              uint16_t sign = SetRightVelocity(vx,vy,vz);
              myprotocolHeader.SetVx(abs(vx));
              myprotocolHeader.SetVy(abs(vy));
              myprotocolHeader.SetVz(abs(vz));
              myprotocolHeader.SetSign(sign);
              myprotocolHeader.SetTimestamp(Simulator::Now ().ToInteger(Time::S));

              m_routingTable.LookupRoute (m_ipv4->GetAddress (1,0).GetBroadcast (),ownEntry);
              ownEntry.SetSeqNo (myprotocolHeader.GetDstSeqno ());
              m_routingTable.Update (ownEntry);
              packet->AddHeader (myprotocolHeader);
            }
          else
            {
              myprotocolHeader.SetDst (i->second.GetDestination ());
              myprotocolHeader.SetDstSeqno ((i->second.GetSeqNo ()));
              myprotocolHeader.SetHopCount (i->second.GetHop () + 1);

              // ADD:将自己的路由表信息转换成header包头的信息。
              myprotocolHeader.SetX(i->second.GetX());
              myprotocolHeader.SetY(i->second.GetY());
              myprotocolHeader.SetZ(i->second.GetZ());
              myprotocolHeader.SetVx(abs(i->second.GetVx()));
              myprotocolHeader.SetVy(abs(i->second.GetVy()));
              myprotocolHeader.SetVz(abs(i->second.GetVz()));
              uint16_t sign = SetRightVelocity(i->second.GetVx(),i->second.GetVy(),i->second.GetVz());
              myprotocolHeader.SetSign(sign);
              myprotocolHeader.SetTimestamp(i->second.GetTimestamp());

              packet->AddHeader (myprotocolHeader);
            }
          NS_LOG_DEBUG ("Forwarding the update for " << i->first);
          NS_LOG_DEBUG ("Forwarding details are, Destination: " << myprotocolHeader.GetDst ()
                                                                << ", SeqNo:" << myprotocolHeader.GetDstSeqno ()
                                                                << ", HopCount:" << myprotocolHeader.GetHopCount ()
                                                                << ", LifeTime: " << i->second.GetLifeTime ().GetSeconds ());
        }
      for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator rmItr = removedAddresses.begin (); rmItr
           != removedAddresses.end (); ++rmItr)
        {
          MyprotocolHeader removedHeader;
          removedHeader.SetDst (rmItr->second.GetDestination ());
          removedHeader.SetDstSeqno (rmItr->second.GetSeqNo () + 1);
          removedHeader.SetHopCount (rmItr->second.GetHop () + 1);
          packet->AddHeader (removedHeader);
          NS_LOG_DEBUG ("Update for removed record is: Destination: " << removedHeader.GetDst ()
                                                                      << " SeqNo:" << removedHeader.GetDstSeqno ()
                                                                      << " HopCount:" << removedHeader.GetHopCount ());
        }
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
  RoutingTableEntry rt (
    /*device=*/ m_lo,  /*dst=*/
    Ipv4Address::GetLoopback (), /*seqno=*/
    0,
    /*iface=*/ Ipv4InterfaceAddress (Ipv4Address::GetLoopback (),Ipv4Mask ("255.0.0.0")),
    /*hops=*/ 0,  /*next hop=*/
    Ipv4Address::GetLoopback (),
    /*lifetime=*/ Simulator::GetMaximumSimulationTime ());
  rt.SetFlag (INVALID);
  rt.SetEntriesChanged (false);
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
  Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()));
  RoutingTableEntry rt (/*device=*/ dev, /*dst=*/ iface.GetBroadcast (), /*seqno=*/ 0,/*iface=*/ iface,/*hops=*/ 0,
                                    /*next hop=*/ iface.GetBroadcast (), /*lifetime=*/ Simulator::GetMaximumSimulationTime ());
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
  m_routingTable.DeleteAllRoutesFromInterface (m_ipv4->GetAddress (i,0));
  m_advRoutingTable.DeleteAllRoutesFromInterface (m_ipv4->GetAddress (i,0));
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
      Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()));
      RoutingTableEntry rt (/*device=*/ dev, /*dst=*/ iface.GetBroadcast (),/*seqno=*/ 0, /*iface=*/ iface,/*hops=*/ 0,
                                        /*next hop=*/ iface.GetBroadcast (), /*lifetime=*/ Simulator::GetMaximumSimulationTime ());
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

void
RoutingProtocol::LookForQueuedPackets ()
{
  NS_LOG_FUNCTION (this);
  Ptr<Ipv4Route> route;
  std::map<Ipv4Address, RoutingTableEntry> allRoutes;
  m_routingTable.GetListOfAllRoutes (allRoutes);
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = allRoutes.begin (); i != allRoutes.end (); ++i)
    {
      RoutingTableEntry rt;
      rt = i->second;
      if (m_queue.Find (rt.GetDestination ()))
        {
          if (rt.GetHop () == 1)
            {
              route = rt.GetRoute ();
              NS_LOG_LOGIC ("A route exists from " << route->GetSource ()
                                                   << " to neighboring destination "
                                                   << route->GetDestination ());
              NS_ASSERT (route != 0);
            }
          else
            {
              RoutingTableEntry newrt;
              m_routingTable.LookupRoute (rt.GetNextHop (),newrt);
              route = newrt.GetRoute ();
              NS_LOG_LOGIC ("A route exists from " << route->GetSource ()
                                                   << " to destination " << route->GetDestination () << " via "
                                                   << rt.GetNextHop ());
              NS_ASSERT (route != 0);
            }
          SendPacketFromQueue (rt.GetDestination (),route);
        }
    }
}

void
RoutingProtocol::SendPacketFromQueue (Ipv4Address dst,
                                      Ptr<Ipv4Route> route)
{
  NS_LOG_DEBUG (m_mainAddress << " is sending a queued packet to destination " << dst);
  QueueEntry queueEntry;
  if (m_queue.Dequeue (dst,queueEntry))
    {
      DeferredRouteOutputTag tag;
      Ptr<Packet> p = ConstCast<Packet> (queueEntry.GetPacket ());
      if (p->RemovePacketTag (tag))
        {
          if (tag.oif != -1 && tag.oif != m_ipv4->GetInterfaceForDevice (route->GetOutputDevice ()))
            {
              NS_LOG_DEBUG ("Output device doesn't match. Dropped.");
              return;
            }
        }
      UnicastForwardCallback ucb = queueEntry.GetUnicastForwardCallback ();
      Ipv4Header header = queueEntry.GetIpv4Header ();
      header.SetSource (route->GetSource ());
      header.SetTtl (header.GetTtl () + 1); // compensate extra TTL decrement by fake loopback routing
      ucb (route,p,header);
      if (m_queue.GetSize () != 0 && m_queue.Find (dst))
        {
          Simulator::Schedule (MilliSeconds (m_uniformRandomVariable->GetInteger (0,100)),
                               &RoutingProtocol::SendPacketFromQueue,this,dst,route);
        }
    }
}

Time
RoutingProtocol::GetSettlingTime (Ipv4Address address)
{
  NS_LOG_FUNCTION ("Calculating the settling time for " << address);
  RoutingTableEntry mainrt;
  Time weightedTime;
  m_routingTable.LookupRoute (address,mainrt);
  if (EnableWST)
    {
      if (mainrt.GetSettlingTime () == Seconds (0))
        {
          return Seconds (0);
        }
      else
        {
          NS_LOG_DEBUG ("Route SettlingTime: " << mainrt.GetSettlingTime ().GetSeconds ()
                                               << " and LifeTime:" << mainrt.GetLifeTime ().GetSeconds ());
          weightedTime = Time (m_weightedFactor * mainrt.GetSettlingTime ().GetSeconds () + (1.0 - m_weightedFactor)
                               * mainrt.GetLifeTime ().GetSeconds ());
          NS_LOG_DEBUG ("Calculated weightedTime:" << weightedTime.GetSeconds ());
          return weightedTime;
        }
    }
  return mainrt.GetSettlingTime ();
}

void
RoutingProtocol::MergeTriggerPeriodicUpdates ()
{
  NS_LOG_FUNCTION ("Merging advertised table changes with main table before sending out periodic update");
  std::map<Ipv4Address, RoutingTableEntry> allRoutes;
  m_advRoutingTable.GetListOfAllRoutes (allRoutes);
  if (allRoutes.size () > 0)
    {
      for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = allRoutes.begin (); i != allRoutes.end (); ++i)
        {
          RoutingTableEntry advEntry = i->second;
          if ((advEntry.GetEntriesChanged () == true) && (!m_advRoutingTable.AnyRunningEvent (advEntry.GetDestination ())))
            {
              if (!(advEntry.GetSeqNo () % 2))
                {
                  advEntry.SetFlag (VALID);
                  advEntry.SetEntriesChanged (false);
                  m_routingTable.Update (advEntry);
                  NS_LOG_DEBUG ("Merged update for " << advEntry.GetDestination () << " with main routing Table");
                }
              m_advRoutingTable.DeleteRoute (advEntry.GetDestination ());
            }
          else
            {
              NS_LOG_DEBUG ("Event currently running. Cannot Merge Routing Tables");
            }
        }
    }
}
}
}
