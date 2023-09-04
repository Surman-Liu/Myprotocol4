/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
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
 */
#include "myprotocol-id-cache.h"
#include <algorithm>

namespace ns3 {
namespace myprotocol {
bool
IdCache::IsDuplicate (Ipv4Address addr, uint16_t timestamp)
{
  Purge ();
  for (std::vector<UniqueId>::const_iterator i = m_idCache.begin ();
       i != m_idCache.end (); ++i)
    {
      if (i->m_context == addr && i->m_timestamp == timestamp)
        {
          return true;
        }
    }
  struct UniqueId uniqueId =
  {
    addr, timestamp, m_lifetime + Simulator::Now ()
  };
  m_idCache.push_back (uniqueId);
  return false;
}
void
IdCache::Purge ()
{
  m_idCache.erase (remove_if (m_idCache.begin (), m_idCache.end (),
                              IsExpired ()), m_idCache.end ());
}

uint32_t
IdCache::GetSize ()
{
  Purge ();
  return m_idCache.size ();
}

}
}
