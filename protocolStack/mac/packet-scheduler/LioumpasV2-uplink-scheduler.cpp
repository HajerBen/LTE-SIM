/*
 * LioumpasV2.cpp
 *
 *  Created on: 16 nov. 2018
 *      Author: hajer
 */

/*
 * rme_uplink-packet_scheduler.cpp
 *
 *  Created on: 10 oct. 2018
 *      Author: hajer
 */

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012
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
 * Author: Hajer Ben Rekhissa
 *
 */

#include "LioumpasV2-uplink-scheduler.h"

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
//#define Allocation
LioumpasV2::LioumpasV2() {
	SetMacEntity(0);
	CreateUsersToSchedule();
}

LioumpasV2::~LioumpasV2() {
	Destroy();
}

double LioumpasV2::ComputeSchedulingMetric(RadioBearer *bearer,
		double spectralEfficiency, int subChannel) {
	double metric;
	return metric;
}
//for H2H application
double LioumpasV2::ComputeSchedulingMetric(UserToSchedule* user,
		int subchannel) {
	double metric;
	int channelCondition = user->m_channelContition.at(subchannel);
	double spectralEfficiency = GetMacEntity()->GetAmcModule()->GetSinrFromCQI(
			channelCondition);
	metric = spectralEfficiency;
	return metric;
}
//for M2M users
double LioumpasV2::ComputeSchedulingMetric(UserToSchedule* user) {
	double metric; //rest delay
	double maxDelay;
	RrcEntity *rrc = user->m_userToSchedule->GetProtocolStack()->GetRrcEntity();
	//GetMacEntity()->GetDevice ()->GetProtocolStack ()
	for (RrcEntity::RadioBearersContainer::iterator it =
			rrc->GetRadioBearerContainer()->begin();
			it != rrc->GetRadioBearerContainer()->end(); it++) {
		RadioBearer *b = (*it);
		Application* app = b->GetApplication();
		//app->SetApplicationID(user->m_userToSchedule->GetIDNetworkNode());
		maxDelay = app->GetQoSParameters()->GetMaxDelay();
	}
	metric = maxDelay - user->m_delay;
	return metric;
}
/*This is an implementation of algorithm 2 reported in
 * A.  Lioumpas  and  A.  Alexiou,  “Uplink  scheduling  for  machine-to-
 * machine  communications  in  lte-based  cellular  systems,
 *
 */
void LioumpasV2::RBsAllocation() {

	UsersToSchedule *users = GetUsersToSchedule();
	UserToSchedule* scheduledUser;
	int nbOfRBs =
			GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();
	int availableRBs;     // No of RB's not allocated
	double delay[users->size()];
	double lowestDelay;	  //lowest delay to identify UE with lowest delay
	int selectedUser;     // user to be selected for allocation
	int requiredPRBs[users->size()];
	double metrics[nbOfRBs][users->size()];
	int selectedPRB;
	bool Allocated[nbOfRBs];
	double bestSinr;
	int unallocatedUsers; // No of users who remain unallocated
	bool ContinueAllocation;
	int MatriceOfAllocation[nbOfRBs];
	int left, right;      // index of left and left PRB's to check
	//initialization
	for (int i = 0; i < nbOfRBs; i++) {
		Allocated[i] = false;
		MatriceOfAllocation[i] = -1;
	}
	ContinueAllocation = true;
	availableRBs = nbOfRBs;
	unallocatedUsers = users->size();


	//create number of required PRB's per scheduled users
	for (int j = 0; j < users->size(); j++) {
		scheduledUser = users->at(j);
#ifdef SCHEDULER_DEBUG
		cout << "\n" << "User " << j << std::endl; // << "CQI Vector";
#endif
//		std::cout << "CQI " << "\n";
		std::vector<double> sinrs;
		for (std::vector<int>::iterator c =
				scheduledUser->m_channelContition.begin();
				c != scheduledUser->m_channelContition.end(); c++) {
			//std::cout << *c << " ";
			sinrs.push_back(GetMacEntity()->GetAmcModule()->GetSinrFromCQI(*c));
		}
		//		std::cout << "\n";
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
		<< " Required RBs " << requiredPRBs[j] << " data to transmit "
		<< scheduledUser->m_dataToTransmit << " TB size "
		<< GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(mcs, 1)
		<< "\n";
#endif
	}

	//Create matrix of flow metrics:delay
	for (int i = 0; i < users->size(); i++) {
		delay[i] = ComputeSchedulingMetric(users->at(i));
	}
	//create a matrix of sinr metrics
	for (int i = 0; i < nbOfRBs; i++) {
		for (int j = 0; j < users->size(); j++) {
			metrics[i][j] = ComputeSchedulingMetric(users->at(j), i);
		}
	}
	for (int ii = 0; ii < users->size(); ii++) {
			std::cout << "Metrics for user "
			<< users->at(ii)->m_userToSchedule->GetIDNetworkNode() << "\n";
			for (int jj = 0; jj < nbOfRBs; jj++) {
				//std::cout  << setw(3) << metrics[jj][ii]/1000 << " ";
				printf("%3d  ", (int) (metrics[jj][ii]));
			}
			std::cout << std::endl;
		}
//sort the MTC devices in ascending order with respect to dk
	/*	for (int i = 0; i < users->size(); i++) {
	 for (int j = 0; j < users->size() - 1; j++) {
	 if (delay[SortedMTC.at(j)] > delay[SortedMTC.at(j + 1)]) {
	 int a;
	 a = SortedMTC.at(j);
	 SortedMTC.at(j) = SortedMTC.at(j + 1);
	 SortedMTC.at(j + 1) = a;
	 }
	 }
	 }*/

	while (availableRBs > 0 && unallocatedUsers > 0) //
	{
		//Search for the user with lowest delay
		lowestDelay = (double) ((1 << 30));
		selectedUser = -1;
		for (int j = 0; j < users->size(); j++) {
			if (users->at(j)->m_listOfAllocatedRBs.size() == 0
					&& requiredPRBs[j] > 0) //only unallocated users requesting some RB's
							{
				if (lowestDelay > delay[j]) {
					selectedUser = j;
					lowestDelay = delay[j];
				}
			}
		}
		ContinueAllocation = true;
		scheduledUser = users->at(selectedUser);
		//step 4 : allocated RBs
		while (availableRBs > 0 && requiredPRBs[selectedUser] > 0
				&& ContinueAllocation) {
			selectedPRB = -1;
			bestSinr = (double) (-(1 << 30));
			for (int i = 0; i < nbOfRBs; i++) {
				if (!Allocated[i]) {
					if (bestSinr < metrics[i][selectedUser]) {
						selectedPRB = i;
						bestSinr = metrics[i][selectedUser];
					}
				}
			}
			//verify if selected PRB is adjacent to already allocated RB for selectedUser
			if (scheduledUser->m_listOfAllocatedRBs.size() > 0){
				left = selectedPRB -1;
				right = selectedPRB+1;
				if (((left > 0) &&(MatriceOfAllocation[left] == selectedUser))
						||((right < nbOfRBs) && (MatriceOfAllocation[right] == selectedUser)))
				{
					ContinueAllocation = true;
				}
				else
					ContinueAllocation = false;
			}
			if (ContinueAllocation)
			{
			scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);
			Allocated[selectedPRB] = true;
			availableRBs--;
			MatriceOfAllocation[selectedPRB] = selectedUser;
			}
		}
		unallocatedUsers--;
		//allocatedUser [selectedUser] = true;
	}

} //end RB Allocation
