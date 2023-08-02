/******************************************************************************
*                        (c) Copyright INESC TEC, 2020.
*            All rights reserved. Content distribution not allowed.
*******************************************************************************

*******************************************************************************/

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/stats-module.h"
#include <fstream>

using namespace ns3;

//------------------------------------------------------
class TimestampTag : public Tag
{
public:
  static TypeId GetTypeId(void);
  virtual TypeId GetInstanceTypeId(void) const;

  virtual uint32_t GetSerializedSize(void) const;
  virtual void Serialize(TagBuffer i) const;
  virtual void Deserialize(TagBuffer i);

  // these are our accessors to our tag structure
  void SetTimestamp(Time time);
  Time GetTimestamp(void) const;

  void Print(std::ostream &os) const;

private:
  Time m_timestamp;

  // end class TimestampTag
};

//----------------------------------------------------------------------
//-- TimestampTag
//------------------------------------------------------
TypeId
TimestampTag::GetTypeId(void)
{
  static TypeId tid = TypeId("TimestampTag")
                          .SetParent<Tag>()
                          .AddConstructor<TimestampTag>()
                          .AddAttribute("Timestamp",
                                        "Some momentous point in time!",
                                        EmptyAttributeValue(),
                                        MakeTimeAccessor(&TimestampTag::GetTimestamp),
                                        MakeTimeChecker());
  return tid;
}
TypeId
TimestampTag::GetInstanceTypeId(void) const
{
  return GetTypeId();
}

uint32_t
TimestampTag::GetSerializedSize(void) const
{
  return 8;
}
void TimestampTag::Serialize(TagBuffer i) const
{
  int64_t t = m_timestamp.GetNanoSeconds();
  i.Write((const uint8_t *)&t, 8);
}
void TimestampTag::Deserialize(TagBuffer i)
{
  int64_t t;
  i.Read((uint8_t *)&t, 8);
  m_timestamp = NanoSeconds(t);
}

void TimestampTag::SetTimestamp(Time time)
{
  m_timestamp = time;
}
Time TimestampTag::GetTimestamp(void) const
{
  return m_timestamp;
}

void TimestampTag::Print(std::ostream &os) const
{
  os << "t=" << m_timestamp;
}
