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
 * Author: Khaled Elsayed <khaled@ieee.org> modified by Hajer Ben Rekhissa
 */

#include "enhanced-uplink-packet-scheduler.h"
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

#define SCHEDULER_DEBUG

EnhancedUplinkPacketScheduler::EnhancedUplinkPacketScheduler() {
	SetMacEntity(0);
	CreateUsersToSchedule();
}

EnhancedUplinkPacketScheduler::~EnhancedUplinkPacketScheduler() {
	Destroy();
}

double EnhancedUplinkPacketScheduler::ComputeSchedulingMetric(
		RadioBearer *bearer, double spectralEfficiency, int subChannel) {
	double metric;
	return metric;
}

double EnhancedUplinkPacketScheduler::ComputeSchedulingMetric(
		UserToSchedule* user, int subchannel) {
	double metric;

	int channelCondition = user->m_channelContition.at(subchannel);
	double spectralEfficiency = GetMacEntity()->GetAmcModule()->GetSinrFromCQI(
			channelCondition);

	metric = spectralEfficiency * 180000;

	return metric;
}

void EnhancedUplinkPacketScheduler::RBsAllocation() {

#ifdef SCHEDULER_DEBUG
	std::cout << " ---- UL RBs Allocation(fme)";
#endif

	UsersToSchedule *users = GetUsersToSchedule();
	UserToSchedule* scheduledUser;
	int nbOfRBs =
			GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();

	int availableRBs;     // No of RB's not allocated
	int selectedUser;     // user to be selected for allocation
	int selectedPRB;      // PRB to be selected for allocation
	double bestMetric;    // best metric to identify user/RB combination
	int left, right;      // index of left and left PRB's to check
	bool Allocated[nbOfRBs];
	double metrics[nbOfRBs][users->size()];
	int requiredPRBs[users->size()];
	bool ContinueRight, ContinueLeft; //To verify if RB+1 is better or worse than RB-1
	bool Verify; // to verify if selected user has the best metric at adjacent RB
	int newUe, lastUe;
	bool allocatedUser[users->size()];
	//some initialization
	availableRBs = nbOfRBs;
	ContinueRight = ContinueLeft = false;
	//allocated = false
	for (int i = 0; i < nbOfRBs; i++)
		Allocated[i] = false;
	//allocatedUser = false; finishAllocation = false
	for (int i = 0; i < users->size(); i++) {
		allocatedUser[i] = false;
	}
	//create a matrix of flow metrics
	for (int i = 0; i < nbOfRBs; i++) {
		for (int j = 0; j < users->size(); j++) {
			metrics[i][j] = ComputeSchedulingMetric(users->at(j), i);
		}
	}
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
		cout << ":  EffSINR = " << effectiveSinr << "  MCS = " << mcs
				<< "Required RBs " << requiredPRBs[j] << "\n";
#endif
	}
#ifdef SCHEDULER_DEBUG
	std::cout << " available RBs " << nbOfRBs << ", users " << users->size()
			<< std::endl;
	for (int ii = 0; ii < users->size(); ii++) {
		std::cout << "Metrics for user "
				<< users->at(ii)->m_userToSchedule->GetIDNetworkNode() << "\n";
		for (int jj = 0; jj < nbOfRBs; jj++) {
			//std::cout  << setw(3) << metrics[jj][ii]/1000 << " ";
			printf("%3d  ", (int) (metrics[jj][ii] / 1000.0));
		}
		std::cout << std::endl;
	}
#endif
	//RBs Allocation
	while (availableRBs > 0) //
	{
		//first Step: find the best user-RB combi
		selectedPRB = -1;
		selectedUser = -1;
		Verify = true;

		bestMetric = (double) (-(1 << 30));
		std::cout << "now= " << Simulator::Init()->Now() << std::endl;
		for (int i = 0; i < nbOfRBs; i++) {
			if (!Allocated[i]) {	//check only unallocated PRBs
				for (int j = 0; j < users->size(); j++) {
					if (users->at(j)->m_listOfAllocatedRBs.size() == 0
							&& requiredPRBs[j] > 0) //only unallocated users requesting some RB's
						if (bestMetric < metrics[i][j]) {
							selectedPRB = i;
							selectedUser = j;
							bestMetric = metrics[i][j];
						}
				}
			}
		}
		std::cout << "**bestMetric  = " << bestMetric / 1000.0
				<< " selected prb " << selectedPRB << " selected User "
				<< selectedUser << std::endl;
		//Step 2
		// Now start allocating for the selected user at the selected PRB
		if (selectedUser != -1) {
			scheduledUser = users->at(selectedUser);
			scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);
			Allocated[selectedPRB] = true;
			left = selectedPRB - 1;
			right = selectedPRB + 1;
			availableRBs--;
			ContinueLeft = false;
			ContinueRight = false;

			//steps 3 and 4
			// search right and left of initial allocation

			if (left >= 0 && Allocated[left] && right < nbOfRBs
					&& Allocated[right])
				break; // nothing is available, since we need to have contiguous allocation
			if ((right < nbOfRBs) && (!Allocated[right])
					&& (((left >= 0)
							&& (metrics[right][selectedUser]
									>= metrics[left][selectedUser])) // right is better than left
					|| (left < 0) || Allocated[left] // OR no more left
					)) {
				ContinueRight = true;
				std::cout << "start Right" << std::endl;
			} else if ((left >= 0) && (!Allocated[left])
					&& (((right < nbOfRBs)
							&& (metrics[left][selectedUser]
									>= metrics[right][selectedUser])) //left better than right
					|| (right >= nbOfRBs) || Allocated[right] // OR no more right
					)) {
				ContinueLeft = true;
				std::cout << "start Left" << std::endl;
			}
			//end step 3 and 4
			//step 5

			while ((scheduledUser->m_listOfAllocatedRBs.size()
					!= requiredPRBs[selectedUser])
					&& (ContinueRight || ContinueLeft) && availableRBs > 0) {
				//initialize allocatedUser[selectedUser] to continue allocation in the opposite side
				if ((users->at(selectedUser)->m_listOfAllocatedRBs.size()
						!= requiredPRBs[selectedUser])
						&& (ContinueLeft || ContinueRight)) {
					lastUe = selectedUser;
					newUe = selectedUser;
					allocatedUser[selectedUser] = false;
				}
				//Right allocation
				while ((availableRBs > 0) && (!allocatedUser[newUe])
						&& (right < nbOfRBs) && (!Allocated[right])
						&& ContinueRight) {
					std::cout << "Continue Right" << std::endl;
					//verify if UE has the best metric at this RB
					for (int j = 0; j < users->size(); j++) {

						if ((j != lastUe) && !allocatedUser[j]
								&& (metrics[right][newUe] < metrics[right][j])) {
							allocatedUser[lastUe] = true;
							newUe = j;
						}
					}
					//the best metric at right doesn't correspond to UE, so RB will be allocated to the new Ue

					lastUe = newUe;
					Allocated[right] = true;
					users->at(newUe)->m_listOfAllocatedRBs.push_back(right);
					right++;
					availableRBs--;

					if (users->at(newUe)->m_listOfAllocatedRBs.size()
							== requiredPRBs[newUe]) {
						allocatedUser[newUe] = true;
						ContinueRight = false;
						ContinueLeft = true;
					}

					if (right == nbOfRBs) {
						allocatedUser[newUe] = true;
						ContinueRight = false;
						ContinueLeft = true;
					}

				}
				//Left Allocation
				while ((availableRBs > 0) && (!allocatedUser[newUe])
						&& (left >= 0) && (!Allocated[left]) && ContinueLeft) {
					std::cout << "Continue Left" << std::endl;
					//verify if UE has the best metric at this RB
					std::cout << "M[" << newUe << "]= " << metrics[left][newUe]
							<< std::endl;
					for (int j = 0; j < users->size(); j++) {
						if ((j != lastUe) && !allocatedUser[j]
								&& (metrics[left][newUe] < metrics[left][j])) {
							//the best metric at right doesn't correspond to UE, so RB will be allocated to the new Ue
							allocatedUser[lastUe] = true;
							newUe = j;
						}
					}

					lastUe = newUe;
					Allocated[left] = true;
					users->at(newUe)->m_listOfAllocatedRBs.push_back(right);
					left--;
					availableRBs--;
					if (users->at(newUe)->m_listOfAllocatedRBs.size()
							== requiredPRBs[newUe]) {
						allocatedUser[newUe] = true;
						ContinueLeft = false;
						ContinueRight = true;
					}
					if (left < 0) {
						allocatedUser[newUe] = true;
						ContinueLeft = false;
						ContinueRight = true;
					}

				}
			}

		}

	}

} //end RB Allocation
