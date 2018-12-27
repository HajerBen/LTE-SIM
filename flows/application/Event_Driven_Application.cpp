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

EventDrivenApplication::EventDrivenApplication() {
	SetApplicationType(Application::APPLICATION_TYPE_EVENT_DRIVEN);
}

EventDrivenApplication::~EventDrivenApplication() {
	Destroy();
}

void EventDrivenApplication::DoStart(void) {
	Simulator::Init()->Schedule(0.0, &EventDrivenApplication::Send, this);
}

void EventDrivenApplication::DoStop(void) {
}

void EventDrivenApplication::ScheduleTransmit(double time) {
	if ((Simulator::Init()->Now() + time) < GetStopTime()) {
		Simulator::Init()->Schedule(time, &EventDrivenApplication::Send, this);
	}
}

void EventDrivenApplication::Send(void) {
	//CREATE A NEW PACKET (ADDING UDP, IP and PDCP HEADERS)
	Packet *packet = new Packet();
	int uid = Simulator::Init()->GetUID();

	packet->SetID(uid);
	packet->SetTimeStamp(Simulator::Init()->Now());
	packet->SetSize(GetSize());

	PacketTAGs *tags = new PacketTAGs();
	tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_EVENT_DRIVEN);
	tags->SetApplicationSize(packet->GetSize());
	packet->SetPacketTags(tags);

	UDPHeader *udp = new UDPHeader(GetClassifierParameters()->GetSourcePort(),
			GetClassifierParameters()->GetDestinationPort());
	packet->AddUDPHeader(udp);

	IPHeader *ip = new IPHeader(GetClassifierParameters()->GetSourceID(),
			GetClassifierParameters()->GetDestinationID());
	packet->AddIPHeader(ip);

	PDCPHeader *pdcp = new PDCPHeader();
	packet->AddPDCPHeader(pdcp);

	Trace(packet);

	GetRadioBearer()->Enqueue(packet);

	ScheduleTransmit (GetInterval ());
	//ScheduleTransmit(PoissonRand());

}

int EventDrivenApplication::GetSize(void) const {
	return m_size;
}

void EventDrivenApplication::SetSize(int size) {
	m_size = size;
}
void EventDrivenApplication::SetInterval(double interval) {
	m_interval = interval;
}

/*double EventDrivenApplication::GetInterval(void) const {
	return m_interval;
}*/

//HB
void EventDrivenApplication::SetLambda(int lambda) {
	m_lambda = lambda;
	PoissonRand();
}
void EventDrivenApplication::PoissonRand() {
	/*float d, x;
	int k, r;
	double interval;

	k = rand() % 50+1;
	x = exp(-GetLambda());
	r = fact(k);
	d = pow(GetLambda(), k) / (float) r;
	interval = d * x;*/

	double duration = (GetStopTime() - GetStartTime())*1000;//en ms
	int smallInterval=  duration / GetLambda();
	int nbEvents = rand()% smallInterval+1;
	 m_interval = duration / nbEvents;
	 std::cout << "nbEvents " << nbEvents << " Interval " << m_interval << std::endl;
}
double EventDrivenApplication::GetInterval(void) const {
		return m_interval;
}
/**find k
 * divide duration sur k
 * prendre un nombre aleatoire dansl'intervalle [
 * */
int EventDrivenApplication::GetLambda() const {
	return m_lambda;
}

/*
int EventDrivenApplication::fact(int nbr) {
	// Factorielle de 1 vaut aussi 1
	if (nbr == 0 || nbr == 1) {
		return 1;
	}
	return (nbr * fact(nbr - 1));
}
*/

/*
 * algorithm poisson random number (Knuth):
    init:
         Let L ← e−λ, k ← 0 and p ← 1.
    do:
         k ← k + 1.
         Generate uniform random number u in [0,1] and let p ← p × u.double
    while p > L.
    return k − 1.
 *
 */
