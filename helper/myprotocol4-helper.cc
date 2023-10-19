#include "myprotocol4-helper.h"
#include "ns3/myprotocol4-routing-protocol.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/ipv4-list-routing.h"

namespace ns3 {
Myprotocol4Helper::~Myprotocol4Helper ()
{
}

Myprotocol4Helper::Myprotocol4Helper () : Ipv4RoutingHelper ()
{
  m_agentFactory.SetTypeId ("ns3::myprotocol4::RoutingProtocol");
}

Myprotocol4Helper*
Myprotocol4Helper::Copy (void) const
{
  return new Myprotocol4Helper (*this);
}

Ptr<Ipv4RoutingProtocol>
Myprotocol4Helper::Create (Ptr<Node> node) const
{
  Ptr<myprotocol4::RoutingProtocol> agent = m_agentFactory.Create<myprotocol4::RoutingProtocol> ();
  node->AggregateObject (agent);
  return agent;
}

void
Myprotocol4Helper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}

}
