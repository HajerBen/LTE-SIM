/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010,2011,2012,2013 TELEMATICS LAB, Politecnico di Bari
 *
 * This file is part of LTE-Sim
 *
 * LTE-Sim is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * LTE-Sim is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LTE-Sim; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Giuseppe Piro <g.piro@poliba.it>
 */

#include "roundrobin-uplink-packet-scheduler.h"
#include "../mac-entity.h"
#include "../../packet/Packet.h"
#include "../../packet/packet-burst.h"
#include "../../../device/NetworkNode.h"
#include "../../../flows/radio-bearer.h"
#include "../../../protocolStack/rrc/rrc-entity.h"
#include "../../../flows/application/Application.h"
#include "../../../device/ENodeB.h"
#include "../../../protocolStack/mac/AMCModule.h"
#include "../../../phy/lte-phy.h"
#include "../../../core/spectrum/bandwidth-manager.h"
#include "../../../core/idealMessages/ideal-control-messages.h"
#include "../../../flows/QoS/QoSParameters.h"
#include "../../../flows/MacQueue.h"
#include "../../../utility/eesm-effective-sinr.h"

//#define SCHEDULER_DEBUG
#define Allocation

RoundRobinUplinkPacketScheduler::RoundRobinUplinkPacketScheduler() {
	SetMacEntity(0);
	CreateUsersToSchedule();

	m_roundRobinId = 0;

}

RoundRobinUplinkPacketScheduler::~RoundRobinUplinkPacketScheduler() {
	Destroy();
}

double RoundRobinUplinkPacketScheduler::ComputeSchedulingMetric(
		RadioBearer *bearer, double spectralEfficiency, int subChannel) {
	double metric;
	return metric;
}

double RoundRobinUplinkPacketScheduler::ComputeSchedulingMetric(
		UserToSchedule* user, int subchannel) {
	double metric;

	int channelCondition = user->m_channelContition.at(subchannel);
	double spectralEfficiency = GetMacEntity()->GetAmcModule()->GetSinrFromCQI(
			channelCondition);

	metric = spectralEfficiency * 180000;

	return metric;
}

void RoundRobinUplinkPacketScheduler::RBsAllocation() {

#ifdef Allocation
	std::cout << "RBsAllocation (RR)" << std::endl;
	std::cout << " ---- RR UL RBs Allocation";
#endif

	UsersToSchedule *users = GetUsersToSchedule();
	UserToSchedule* scheduledUser;
	int nbOfRBs =
			GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();
	int nbrOfScheduledUsers;
	nbrOfScheduledUsers = 0;
	int requiredPRBs[users->size()];

	//RBs allocation
	int nbPrbToAssign = 5;
	int stop_nbOfRBs = nbOfRBs;
	if ((nbOfRBs / users->size()) > nbPrbToAssign) {
		nbPrbToAssign = ceil(nbOfRBs / users->size());
		stop_nbOfRBs = nbPrbToAssign * users->size();
	}

#ifdef Allocation
	std::cout << "  PRB to assign " << nbOfRBs << ", PRB for user "

	<< nbPrbToAssign << std::endl;
#endif

	int s = 0;

	//create number of required PRB's per scheduled users
	for (int j = 0; j < users->size(); j++) {
		scheduledUser = users->at(j);
#ifdef SCHEDULER_DEBUG
		cout << "\n" << "User " << j; // << "CQI Vector";
#endif

		std::vector<double> sinrs;
		for (std::vector<int>::iterator c =
				scheduledUser->m_channelContition.begin();
				c != scheduledUser->m_channelContition.end(); c++) {
			//cout << " CQI " << *c << " ";
			sinrs.push_back(GetMacEntity()->GetAmcModule()->GetSinrFromCQI(*c));
		}

		double effectiveSinr = GetEesmEffectiveSinr(sinrs);

		int mcs = GetMacEntity()->GetAmcModule()->GetMCSFromCQI(
				GetMacEntity()->GetAmcModule()->GetCQIFromSinr(effectiveSinr));
		scheduledUser->m_selectedMCS = mcs;
		requiredPRBs[j] = (floor)(
				scheduledUser->m_dataToTransmit
						/ (GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(mcs,
								1) / 8));

#ifdef SCHEDULER_DEBUG
		cout << " EffSINR = " << effectiveSinr << "  MCS = " << mcs
				<< "Required RBs " << requiredPRBs[j] << "\n";
#endif
	}

	while (s < stop_nbOfRBs) {
		if (m_roundRobinId >= users->size())
			m_roundRobinId = 0; //restart again from the beginning

		if (requiredPRBs[m_roundRobinId] > 0) {
			scheduledUser = users->at(m_roundRobinId);
#ifdef SCHEDULER_DEBUG
			std::cout << "user to schedule "
					<< scheduledUser->m_userToSchedule->GetIDNetworkNode()
					<< std::endl;
#endif
			std::vector<double> sinrs;
			for (int i = 0; i < nbPrbToAssign; i++) {
				double chCondition = scheduledUser->m_channelContition.at(
						s + i);
				double sinr = chCondition;
				sinrs.push_back(sinr);
				scheduledUser->m_listOfAllocatedRBs.push_back(s + i);
			}

			int tbs = ((GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(
					scheduledUser->m_selectedMCS, nbPrbToAssign)) / 8);
			scheduledUser->m_transmittedData = tbs;

			s = s + nbPrbToAssign;
			//HB

#ifdef SCHEDULER_DEBUG
			printf(
					"Scheduled User = %d mcs = %d Required RB's = %d Allocated RB's= %d\n",
					scheduledUser->m_userToSchedule->GetIDNetworkNode(),
					scheduledUser->m_selectedMCS, requiredPRBs[m_roundRobinId],
					scheduledUser->m_listOfAllocatedRBs.size());
			for (int i = 0; i < scheduledUser->m_listOfAllocatedRBs.size(); i++)
				printf("%d ", scheduledUser->m_listOfAllocatedRBs.at(i));
			printf("\n------------------\n");
#endif

//Calculate power
			scheduledUser->m_power = CalculatePower(
					scheduledUser->m_listOfAllocatedRBs.size(), scheduledUser);
			//number of scheduled users per TTI
			if (scheduledUser->m_listOfAllocatedRBs.size() > 0)
				nbrOfScheduledUsers++;

			//end HB
			m_roundRobinId++;
		}
		else
			m_roundRobinId++;
		//verify if all required PRB>0
		int j = 0;
for (int i =0 ; i< users->size()&&requiredPRBs[i]==0; i++){
	j++;
}
if (j==users->size())
		break;
	}
#ifdef SCHEDULER_DEBUG
	std::cout << "number of scheduled users per TTI " << nbrOfScheduledUsers
			<< std::endl;
#endif
}

