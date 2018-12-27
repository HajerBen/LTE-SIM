/*
 * LioumpasV2.cpp
 *
 *  Created on: 16 nov. 2018
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
double LioumpasV2::ComputeSchedulingMetric(UserToSchedule* user,
		RadioBearer *bearer, int subChannel) {
	double metric;
	if ((bearer->GetApplication()->GetApplicationType()
			== Application::APPLICATION_TYPE_TRACE_BASED)
			|| (bearer->GetApplication()->GetApplicationType()
					== Application::APPLICATION_TYPE_CBR)
			|| (bearer->GetApplication()->GetApplicationType()
					== Application::APPLICATION_TYPE_VOIP)) {
		int channelCondition = user->m_channelContition.at(subChannel);
		double spectralEfficiency =
				GetMacEntity()->GetAmcModule()->GetSinrFromCQI(
						channelCondition);
		metric = (spectralEfficiency * 180000.)
				/ bearer->GetAverageTransmissionRate();


	} else {
		double maxDelay = bearer->GetQoSParameters()->GetMaxDelay();
		metric = maxDelay - user->m_delay;
	}
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
	int availableH2HRBs;
	double delay[users->size()];
	double lowestDelay;	  //lowest delay to identify UE with lowest delay
	int selectedUser;     // user to be selected for allocation
	int requiredPRBs[users->size()];
	double metrics[nbOfRBs][users->size()];
	int selectedPRB;
	bool Allocated[nbOfRBs];
	double bestSinr;
//	int unallocatedUsers; // No of users who remain unallocated
	int unallocatedH2HUsers;     // No of H2H users which remain unallocated
	int unallocatedM2MUsers;
	bool ContinueAllocation;
	int MatriceOfAllocation[nbOfRBs];
	int left, right;      // index of left and left PRB's to check
	bool ContinueRight;
	bool ContinueLeft;
	double bestMetric;    // best metric to identify user/RB combination
	int nbrOfScheduledUsers;
	//initialization
	for (int i = 0; i < nbOfRBs; i++) {
		Allocated[i] = false;
		MatriceOfAllocation[i] = -1;
	}

	ContinueAllocation = true;
	availableRBs = nbOfRBs;
	availableH2HRBs =  nbOfRBs;


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

	std::vector<UserToSchedule*> M2MUsers;
	std::vector<UserToSchedule*> H2HUsers;

	//Differentiate between M2M and H2H devices and create metrics
	for (int j = 0; j < users->size(); j++) {
		scheduledUser = users->at(j);
		if (users->at(j)->m_userToSchedule->GetIDNetworkNode() >= 100
				&& users->at(j)->m_userToSchedule->GetIDNetworkNode() < 500) {
			RrcEntity *rrc = scheduledUser->m_userToSchedule->GetProtocolStack()->GetRrcEntity();
			H2HUsers.push_back(users->at(j));
			int app =0;
			for (RrcEntity::RadioBearersContainer::iterator it =
					rrc->GetRadioBearerContainer()->begin();
					it != rrc->GetRadioBearerContainer()->end(); it++) {
				RadioBearer *b = (*it);
				Application* app = b->GetApplication();
				for (int i = 0; i < nbOfRBs; i++) {
					metrics[i][j] = ComputeSchedulingMetric(scheduledUser, b,
							i);
				}
			}
		}
		else {
			M2MUsers.push_back(users->at(j));
			delay[j] = ComputeSchedulingMetric(users->at(j));
			//create a matrix of sinr metrics
			for (int i = 0; i < nbOfRBs; i++) {
				metrics[i][j] = ComputeSchedulingMetric(users->at(j), i);
			}
		}
	}
	unallocatedH2HUsers = H2HUsers.size();
	unallocatedM2MUsers = M2MUsers.size();
#ifdef SCHEDULER_DEBUG
	for (int ii = 0; ii < H2HUsers.size(); ii++) {
			std::cout << "Metrics for user "
			<< users->at(ii)->m_userToSchedule->GetIDNetworkNode() << "\n";
			for (int jj = 0; jj < nbOfRBs; jj++) {
				printf("%3d  ", (int) (metrics[jj][ii]));
			}
			std::cout << std::endl;
		}
#endif

//Start allocation of H2H devices
	while (availableH2HRBs > 0 && unallocatedH2HUsers > 0) {
		// First step: find the best user-RB combo
		selectedPRB = -1;
		selectedUser = -1;
		ContinueRight = true;
		ContinueLeft = true;
		bestMetric = (double) (-(1 << 30));

		//Search for the best metric
		for (int i = 0; i < nbOfRBs; i++) {
			if (!Allocated[i]) { // check only unallocated PRB's
				for (int j = 0; j < H2HUsers.size(); j++) {
					if (H2HUsers.at(j)->m_listOfAllocatedRBs.size() == 0
							&& requiredPRBs[j] > 0) //only unallocated users requesting some RB's
						if (bestMetric < metrics[i][j]) {
							selectedPRB = i;
							selectedUser = j;
							bestMetric = metrics[i][j];
						}
				}

			}
		}
#ifdef SCHEDULER_DEBUG
		std::cout << "**bestMetric  = " << bestMetric
		<< " selected prb " << selectedPRB << " selected User "
		<< selectedUser << std::endl;
#endif
		if (selectedUser != -1) {
			scheduledUser = H2HUsers.at(selectedUser);
			scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);
			Allocated[selectedPRB] = true;
			left = selectedPRB - 1;
			right = selectedPRB + 1;
			availableH2HRBs--;
			availableRBs--;
			MatriceOfAllocation[selectedPRB] = H2HUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();

			if (left >= 0 && Allocated[left] && right < nbOfRBs
					&& Allocated[right])
				break; // nothing is available, since we need to have contiguous allocation

			while ((scheduledUser->m_listOfAllocatedRBs.size()
					!= requiredPRBs[selectedUser])
					&& (ContinueRight || ContinueLeft) && availableH2HRBs > 0) {
				//start right allocation
				if (right < nbOfRBs && !Allocated[right]) {
					//Verify if the best metric at right doesn't correspond to UE
					for (int j = 0; j < H2HUsers.size(); j++) {

						if ((j != selectedUser) && H2HUsers.at(j)->m_listOfAllocatedRBs.size() == 0
								&& requiredPRBs[j] > 0
								&& (metrics[right][selectedUser]
										< metrics[right][j])) {
							ContinueRight = false;
							break;
						}
					}
					if (ContinueRight) {
						Allocated[right] = true;
						H2HUsers.at(selectedUser)->m_listOfAllocatedRBs.push_back(
								right);
						MatriceOfAllocation[right] = H2HUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();
						right++;
						availableH2HRBs--;
						availableRBs--;
						//std::cout << "right allocation" << std::endl;
					}
				} else
					ContinueRight = false;
				//Start Left allocation
				if (left >= 0 && !Allocated[left]) {
					for (int j = 0; j < H2HUsers.size(); j++) {
						if ((j != selectedUser) && H2HUsers.at(j)->m_listOfAllocatedRBs.size() == 0
								&& requiredPRBs[j] > 0
								&& (metrics[left][selectedUser]
										< metrics[left][j])) {
							ContinueLeft = false;
							break;
						}
					}
					if (ContinueLeft) {
						Allocated[left] = true;
						MatriceOfAllocation[left] = H2HUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();;
						H2HUsers.at(selectedUser)->m_listOfAllocatedRBs.push_back(
								left);
						left--;
						availableH2HRBs--;
						availableRBs--;
					}
				} else
					ContinueLeft = false;
			}
				unallocatedH2HUsers--;
		} //end if
		else
			//no more users to allocate
			break;
#ifdef SCHEDULER_DEBUG
		std::cout << "unallocatedUsers " << unallocatedH2HUsers << std::endl;
#endif
	} //end While

	//*********Start M2M Allocation *****/

	for (int j = 0; j < M2MUsers.size(); j++) {
		delay[j] = ComputeSchedulingMetric(users->at(j));
		//create a matrix of sinr metrics
		for (int i = 0; i < nbOfRBs; i++) {
			metrics[i][j] = ComputeSchedulingMetric(M2MUsers.at(j), i);
		}
#ifdef SCHEDULER_DEBUG
		std::cout << "j " << j << " delay = " << delay[j] << std::endl;
		std::cout << "metric of " << j << std::endl;
		for (int jj = 0; jj < nbOfRBs; jj++) {
			//std::cout  << setw(3) << metrics[jj][ii]/1000 << " ";
			printf("%3d  ", (int) (metric[jj][j]));
		}
		std::cout << std::endl;
#endif
		}

	while (availableRBs > 0 && unallocatedM2MUsers > 0) //
	{
		//Search for the user with lowest delay
		lowestDelay = (double) ((1 << 30));
		selectedUser = -1;
		for (int j = 0; j < M2MUsers.size(); j++) {
			if (M2MUsers.at(j)->m_listOfAllocatedRBs.size() == 0
					&& requiredPRBs[j] > 0) //only unallocated users requesting some RB's
							{
				if (lowestDelay > delay[j]) {
					selectedUser = j;
					lowestDelay = delay[j];
				}
			}
		}
		if (selectedUser != -1) {
		ContinueAllocation = true;
		scheduledUser = M2MUsers.at(selectedUser);
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
			if (scheduledUser->m_listOfAllocatedRBs.size() > 0) {
				left = selectedPRB - 1;
				right = selectedPRB + 1;
				if (((left > 0) && (MatriceOfAllocation[left] == M2MUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode()))
						|| ((right < nbOfRBs)
								&& (MatriceOfAllocation[right] == M2MUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode()))) {
					ContinueAllocation = true;
				} else
					ContinueAllocation = false;
			}
			if (ContinueAllocation) {
				scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);
				Allocated[selectedPRB] = true;
				availableRBs--;
				MatriceOfAllocation[selectedPRB] = M2MUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();
			}
		}
		unallocatedM2MUsers--;
				}else {
				break;
			}
	}

#ifdef Allocation
for (int i =0; i < nbOfRBs; i++){

		 std::cout << "Mallocation[" << i << "] =" << MatriceOfAllocation[i]
		 << std::endl;

}
#endif
UserToSchedule* scheduledUser1;
//Calculate power
	for (int j = 0; j < users->size(); j++) {

		scheduledUser1 = users->at(j);
		scheduledUser1->m_transmittedData =
				GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(
						scheduledUser1->m_selectedMCS,
						scheduledUser1->m_listOfAllocatedRBs.size()) / 8;


			scheduledUser1->m_power = CalculatePower(
					scheduledUser1->m_listOfAllocatedRBs.size(),
					scheduledUser1);
#ifdef SCHEDULER_DEBUG
		printf(
				"Scheduled User = %d mcs = %d Required RB's = %d Allocated RB's= %d\n",
				scheduledUser1->m_userToSchedule->GetIDNetworkNode(),
				scheduledUser1->m_selectedMCS, requiredPRBs[j],
				scheduledUser1->m_listOfAllocatedRBs.size());
		for (int i = 0; i < scheduledUser1->m_listOfAllocatedRBs.size(); i++)
		printf("%d ", scheduledUser1->m_listOfAllocatedRBs.at(i));
		printf("\n------------------\n");
#endif
		//number of scheduled users per TTI
		if (scheduledUser1->m_listOfAllocatedRBs.size() > 0)
			nbrOfScheduledUsers++;
	}
	//std::cout << "number of scheduled users per TTI " << nbrOfScheduledUsers << std::endl;
} //end RB Allocation
