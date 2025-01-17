#ifndef MYPROTOCOL4_ROUTING_PROTOCOL_H
#define MYPROTOCOL4_ROUTING_PROTOCOL_H

#include "myprotocol4-rtable.h"
#include "myprotocol4-packet.h"
#include "myprotocol4-id-cache.h"
#include "myprotocol4-rqueue.h"
#include "ns3/node.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/output-stream-wrapper.h"
// 添加移动模型
#include "ns3/mobility-model.h"
// 添加位置服务
#include "ns3/god.h"
#include<cmath>

namespace ns3 {
namespace myprotocol4 {

class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  static TypeId GetTypeId (void);
  static const uint32_t MYPROTOCOL_PORT;

  RoutingProtocol ();
  virtual
  ~RoutingProtocol ();
  virtual void
  DoDispose ();

  // From Ipv4RoutingProtocol
  Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr);
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
  // ADD:是否使用恢复策略
  bool m_enableRecoveryMode;
  // ADD:是都使用queue
  bool m_enableQueue;
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
  /// Unicast callback for own packets
  UnicastForwardCallback m_scb;
  /// Error callback for own packets
  ErrorCallback m_ecb;

  // ADD：初始化id-cache需要的相关时间变量
  uint32_t m_netDiameter;             ///< Net diameter measures the maximum possible number of hops between two nodes in the network
  Time m_nodeTraversalTime;
  Time m_netTraversalTime;             ///< Estimate of the average net traversal time.
  Time m_pathDiscoveryTime;            ///< Estimate of maximum time needed to find route in network.

  uint16_t m_lastSendTime;        //上次发送更新包的时间
  Vector m_lastSendPos;
  Vector m_lastSendVelocity;
  uint16_t m_maxIntervalTime;    //最大不发送更新包的时间间隔

  // ADD:id-cache
  IdCache m_idCache;

  uint32_t m_maxQueueLen;              ///< The maximum number of packets that we allow a routing protocol to buffer.
  Time m_maxQueueTime;
  // ADD:queue
  RequestQueue m_queue;
  // ADD:节点的通信范围
  uint16_t m_transRange;
  // ADD：扩大范围因子
  float m_scaleFactor;

  // 控制包扩散的范围
  uint16_t m_ttl;

  // ADD：位置服务，用来统计位置误差
  Ptr<LocationService> m_locationService;
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

  void
  SendUpdate ();

  void SendPacketFromQueue (Ipv4Address dst, Ptr<Ipv4Route> route, DataHeader dataHeader);

  void
  Drop (Ptr<const Packet>, const Ipv4Header &, Socket::SocketErrno);

  // ADD:定期检查速度、方向的变化
  void
  CheckChange ();

  // ADD:转换速度符号的两个函数。sign：记录速度是否为负数，0:都不是负数，1:X轴速度为负，2:Y轴速度为负，3:Z轴速度为负,4：xy为负数，5：xz为负数，6：yz为负数，7：全部都是负数
  Vector GetRightVelocity(uint16_t vx, uint16_t vy, uint16_t vz, uint16_t sign);

  uint16_t SetRightVelocity(int16_t vx, int16_t vy, int16_t vz);

  /// ADD： If route exists and valid, forward packet.
  bool Forwarding (Ptr<const Packet> p, const Ipv4Header & header, UnicastForwardCallback ucb, ErrorCallback ecb);

  // ADD:恢复模式
  Ipv4Address RecoveryMode(std::map<Ipv4Address, RoutingTableEntry> & neighborTable);

  // ADD:定时检查速度方向变化的计时器
  Timer m_checkChangeTimer;

  /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;
};

}
}

#endif 
