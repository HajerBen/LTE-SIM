/*
 * Time_Driven_Application.cpp
 *
 *  Created on: 17 nov. 2018
 *      Author: hajer
 */

#include "Time_Driven_Application.h"
#include <cstdlib>
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer.h"
#include "../QoS/QoSParameters.h"

TimeDrivenApplication::TimeDrivenApplication()
{
  SetApplicationType (Application::APPLICATION_TYPE_TIME_DRIVEN);
}

TimeDrivenApplication::~TimeDrivenApplication()
{
  Destroy ();
}

void
TimeDrivenApplication::DoStart (void)
{
  Simulator::Init()->Schedule(0.0, &TimeDrivenApplication::Send, this);
}

void
TimeDrivenApplication::DoStop (void)
{
}

void
TimeDrivenApplication::ScheduleTransmit (double time)
{
  if ( (Simulator::Init()->Now () + time) < GetStopTime () )
    {
      Simulator::Init()->Schedule(time, &TimeDrivenApplication::Send, this);

    }
}

void
TimeDrivenApplication::Send (void)
{

  //CREATE A NEW PACKET (ADDING UDP, IP and PDCP HEADERS)
  Packet *packet = new Packet ();
  int uid = Simulator::Init()->GetUID ();

  packet->SetID(uid);
  packet->SetTimeStamp (Simulator::Init()->Now ());
  packet->SetSize (GetSize ());

  PacketTAGs *tags = new PacketTAGs ();
  tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_TIME_DRIVEN);
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
TimeDrivenApplication::GetSize (void) const
{
  return m_size;
}

void
TimeDrivenApplication::SetSize(int size)
{
  m_size = size;
}
void
TimeDrivenApplication::SetInterval(double interval)
{
  m_interval = interval;
}

double
TimeDrivenApplication::GetInterval (void) const
{
  return m_interval;
}



