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

#ifndef MYPROTOCOL_ROUTING_PROTOCOL_H
#define MYPROTOCOL_ROUTING_PROTOCOL_H

#include "myprotocol-rtable.h"
#include "myprotocol-packet.h"
#include "myprotocol-id-cache.h"
#include "ns3/node.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/output-stream-wrapper.h"
// 添加移动模型
#include "ns3/mobility-model.h"
#include<cmath>

namespace ns3 {
namespace myprotocol {

// ADD:无人机运动的信息
struct Information
{
  float speed;
  float thetaXY;
  float thetaZ;
};

/**
 * \ingroup myprotocol
 * \brief myprotocol routing protocol.
 */
class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  static const uint32_t MYPROTOCOL_PORT;

  /// c-tor
  RoutingProtocol ();
  virtual
  ~RoutingProtocol ();
  virtual void
  DoDispose ();

  // ADD：
  void SetEnableAdaptiveUpdate (bool f){
    m_enableAdaptiveUpdate = f;
  }
  bool GetEnableAdaptiveUpdate () const{
    return m_enableAdaptiveUpdate;
  }

  // From Ipv4RoutingProtocol
  Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr);
  /**
   * Route input packet
   * \param p The packet
   * \param header The IPv4 header
   * \param idev The device
   * \param ucb The unicast forward callback
   * \param mcb The multicast forward callback
   * \param lcb The local deliver callback
   * \param ecb The error callback
   * \returns true if successful
   */
  bool RouteInput (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev, UnicastForwardCallback ucb,
                   MulticastForwardCallback mcb, LocalDeliverCallback lcb, ErrorCallback ecb);
  virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const;
  virtual void NotifyInterfaceUp (uint32_t interface);
  virtual void NotifyInterfaceDown (uint32_t interface);
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);

  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);

private:
  // Protocol parameters.
  /// PeriodicUpdateInterval specifies the periodic time interval between which the a node broadcasts
  /// its entire routing table.
  Time m_periodicUpdateInterval;

  // ADD:是否启用自适应更新
  bool m_enableAdaptiveUpdate;
  // ADD: 检查改变的时间周期  
  Time m_checkChangeInterval;   //检查改变的时间周期  

  /// Nodes IP address
  Ipv4Address m_mainAddress;
  /// IP protocol
  Ptr<Ipv4> m_ipv4;
  /// Raw socket per each IP interface, map socket -> iface address (IP + mask)
  std::map<Ptr<Socket>, Ipv4InterfaceAddress> m_socketAddresses;
  /// Loopback device used to defer route requests until a route is found
  Ptr<NetDevice> m_lo;
  /// Main Routing table for the node
  RoutingTable m_routingTable;
  /// Flag that is used to enable or disable buffering
  bool EnableBuffering;
  /// Unicast callback for own packets
  UnicastForwardCallback m_scb;
  /// Error callback for own packets
  ErrorCallback m_ecb;

  // ADD：记录上次控制包中的方向和速度
  struct Information m_recordInformation;
  struct Information m_sendInformation;
  uint16_t m_thetaThreshold;    //方向改变阈值
  uint16_t m_speedThreshold;    //速度改变阈值

  // ADD：初始化id-cache需要的相关时间变量
  uint32_t m_netDiameter;             ///< Net diameter measures the maximum possible number of hops between two nodes in the network
  /**
   *  NodeTraversalTime is a conservative estimate of the average one hop traversal time for packets
   *  and should include queuing delays, interrupt processing times and transfer times.
   */
  Time m_nodeTraversalTime;
  Time m_netTraversalTime;             ///< Estimate of the average net traversal time.
  Time m_pathDiscoveryTime;            ///< Estimate of maximum time needed to find route in network.

  uint16_t m_lastSendTime;        //上次发送更新包的时间
  bool m_ifChangeLastTime;    //在上一次检查中是否发送了更新包

  // ADD:id-cache
  IdCache m_idCache;

private:
  /// Start protocol operation
  void
  Start ();
  /**
   * Queue packet until we find a route
   * \param p the packet to route
   * \param header the Ipv4Header
   * \param ucb the UnicastForwardCallback function
   * \param ecb the ErrorCallback function
   */
  void
  DeferredRouteOutput (Ptr<const Packet> p, const Ipv4Header & header, UnicastForwardCallback ucb, ErrorCallback ecb);
  /**
   * Find socket with local interface address iface
   * \param iface the interface
   * \returns the socket
   */
  Ptr<Socket>
  FindSocketWithInterfaceAddress (Ipv4InterfaceAddress iface) const;

  // Receive myprotocol control packets
  /**
   * Receive and process myprotocol control packet
   * \param socket the socket for receiving myprotocol control packets
   */
  void
  RecvMyprotocol (Ptr<Socket> socket);
  /// Send packet
  void
  Send (Ptr<Ipv4Route>, Ptr<const Packet>, const Ipv4Header &);
  /**
   * Create loopback route for given header
   *
   * \param header the IP header
   * \param oif the device
   * \returns the route
   */
  Ptr<Ipv4Route>
  LoopbackRoute (const Ipv4Header & header, Ptr<NetDevice> oif) const;
  /**
   * Get settlingTime for a destination
   * \param dst - destination address
   * \return settlingTime for the destination if found
   */
  // Time
  // GetSettlingTime (Ipv4Address dst);
  /// Sends trigger update from a node
  // void
  // SendTriggeredUpdate ();
  /// Broadcasts the entire routing table for every PeriodicUpdateInterval
  void
  SendPeriodicUpdate ();
  /// Merge periodic updates
  // void
  // MergeTriggerPeriodicUpdates ();
  /// Notify that packet is dropped for some reason
  void
  Drop (Ptr<const Packet>, const Ipv4Header &, Socket::SocketErrno);

  // ADD:定期检查速度、方向的变化
  void
  CheckChange ();

  // ADD: 计算运动信息
  void
  CalculateInformation (struct Information &information);

  // ADD:转换速度符号的两个函数。sign：记录速度是否为负数，0:都不是负数，1:X轴速度为负，2:Y轴速度为负，3:Z轴速度为负,4：xy为负数，5：xz为负数，6：yz为负数，7：全部都是负数
  Vector GetRightVelocity(uint16_t vx, uint16_t vy, uint16_t vz, uint16_t sign){
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

  uint16_t SetRightVelocity(int16_t vx, int16_t vy, int16_t vz){
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

  /// ADD： If route exists and valid, forward packet.
  bool Forwarding (Ptr<const Packet> p, const Ipv4Header & header, UnicastForwardCallback ucb, ErrorCallback ecb);

  // ADD:恢复模式
  Ipv4Address RecoveryMode(Ptr<Packet> p, Ipv4Header header);

  /// Timer to trigger periodic updates from a node
  Timer m_periodicUpdateTimer;

  // ADD:定时检查速度方向变化的计时器
  Timer m_checkChangeTimer;

  /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;
};

}
}

#endif /* MYPROTOCOL_ROUTING_PROTOCOL_H */
