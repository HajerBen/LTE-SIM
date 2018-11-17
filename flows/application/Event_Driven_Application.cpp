/*
 * EventDrivenApplication.cpp
 *
 *  Created on: 17 nov. 2018
 *      Author: hajer
 */

#include <cstdlib>
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer.h"
#include "../QoS/QoSParameters.h"
#include "Event_Driven_Application.h"

EventDrivenApplication::EventDrivenApplication()
{
  SetApplicationType (Application::APPLICATION_TYPE_EVENT_DRIVEN);
}

EventDrivenApplication::~EventDrivenApplication()
{
  Destroy ();
}

void
EventDrivenApplication::DoStart (void)
{
  Simulator::Init()->Schedule(0.0, &EventDrivenApplication::Send, this);
}

void
EventDrivenApplication::DoStop (void)
{
}

void
EventDrivenApplication::ScheduleTransmit (double time)
{
  if ( (Simulator::Init()->Now () + time) < GetStopTime () )
    {
      Simulator::Init()->Schedule(time, &EventDrivenApplication::Send, this);
    }
}

void
EventDrivenApplication::Send (void)
{
  //CREATE A NEW PACKET (ADDING UDP, IP and PDCP HEADERS)
  Packet *packet = new Packet ();
  int uid = Simulator::Init()->GetUID ();

  packet->SetID(uid);
  packet->SetTimeStamp (Simulator::Init()->Now ());
  packet->SetSize (GetSize ());

  PacketTAGs *tags = new PacketTAGs ();
  tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_EVENT_DRIVEN);
  tags->SetApplicationSize (packet->GetSize ());
  packet->SetPacketTags(tags);


  UDPHeader *udp = new UDPHeader (GetClassifierParameters ()->GetSourcePort (),
		                          GetClassifierParameters ()->GetDestinationPort ());
  packet->AddUDPHeader (udp);

  IPHeader *ip = new IPHeader (GetClassifierParameters ()->GetSourceID (),
                               GetClassifierParameters ()->GetDestinationID ());
  packet->AddIPHeader (ip);

  PDCPHeader *pdcp = new PDCPHeader ();
  packet->AddPDCPHeader (pdcp);

  Trace (packet);

  GetRadioBearer()->Enqueue (packet);

  ScheduleTransmit (GetInterval ());


}


int
EventDrivenApplication::GetSize (void) const
{
  return m_size;
}

void
EventDrivenApplication::SetSize(int size)
{
  m_size = size;
}
void
EventDrivenApplication::SetInterval(double interval)
{
	  m_interval = interval;
}

double
EventDrivenApplication::GetInterval (void) const
{
  return m_interval;
}



