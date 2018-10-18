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

double EnhancedUplinkPacketScheduler::ComputeSchedulingMetric(RadioBearer *bearer,
		double spectralEfficiency, int subChannel) {
	double metric;
	return metric;
}

double EnhancedUplinkPacketScheduler::ComputeSchedulingMetric(UserToSchedule* user,
		int subchannel) {
	double metric;

	int channelCondition = user->m_channelContition.at(subchannel);
	double spectralEfficiency = GetMacEntity()->GetAmcModule()->GetSinrFromCQI(
			channelCondition);

	metric = spectralEfficiency * 180000;

	return metric;
}

void EnhancedUplinkPacketScheduler::RBsAllocation() {

#ifdef SCHEDULER_DEBUG
	std::cout << " ---- UL RBs Allocation(rme)";
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
	bool allocationMade;
	double metrics[nbOfRBs][users->size()];
	int requiredPRBs[users->size()];
	int MAllocation[nbOfRBs];
	bool ContinueRight, ContinueLeft; //To verify if RB+1 is better or worse than RB-1
	bool VerifyRight, VerifyLeft; // to verify if selected user has the best metric at adjacent RB
	int newUeRight, newUeLeft, lastUeRight, lastUeLeft;
	bool allocatedUser[users->size()];
	bool finishAllocation[users->size()];
	//some initialization
	availableRBs = nbOfRBs;
	ContinueRight = ContinueLeft = false;
	//allocated = false
	for (int i = 0; i < nbOfRBs; i++)
		Allocated[i] = false;
	//allocatedUser = false; finishAllocation = false
	for (int i = 0; i < users->size(); i++) {
		allocatedUser[i] = false;
		finishAllocation[i] = false;
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
		VerifyLeft = true;
		VerifyRight = true;
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
			MAllocation[selectedPRB] = selectedUser;
			allocationMade = true;
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
			lastUeRight = selectedUser;
			newUeRight = selectedUser;
			newUeLeft = selectedUser;
			lastUeLeft = selectedUser;
			while ((users->at(selectedUser)->m_listOfAllocatedRBs.size()
					!= requiredPRBs[selectedUser])
					&& (ContinueRight || ContinueLeft) && availableRBs > 0) {
				if ( (users->at(selectedUser)->m_listOfAllocatedRBs.size()
								!= requiredPRBs[selectedUser]))
					allocatedUser[selectedUser] = false;
				//rightAllocation
				for (int i = 1;
						i < requiredPRBs[newUeRight] && ContinueRight
								&& availableRBs > 0
								&& (!allocatedUser[newUeRight])
								&& allocationMade && (VerifyRight)
								&& (right < nbOfRBs) && (!Allocated[right]);
						i++) {
					allocationMade = false;
					std::cout << "Continue Right" << std::endl;
					for (int j = 0; j < users->size(); j++) {
						if ((j != lastUeRight) && !allocatedUser[j]
								&& (metrics[right][newUeRight]
										< metrics[right][j])) {

							VerifyRight = false;
							newUeRight = j;

						}
					}
					//the best metric at right correspond to UE, so UeRight will be allocated to it
					if (VerifyRight) {
						Allocated[right] = true;
						users->at(newUeRight)->m_listOfAllocatedRBs.push_back(
								right);
						MAllocation[right] = newUeRight;
						right++;
						allocationMade = true;
						availableRBs--;
						std::cout << "allocated right" << std::endl;
						printf(
								"Scheduled User = %d  Required RB's = %d Allocated RB's= %d\n",
								newUeRight, requiredPRBs[newUeRight],
								users->at(newUeRight)->m_listOfAllocatedRBs.size());
					}
					//the adjacent RB don't correspond to selected User so RB will be allocated to the new UE
					else {
						Allocated[right] = true;
						allocatedUser[lastUeRight] = true;
						lastUeRight = newUeRight;
						VerifyRight = true;
						MAllocation[right] = newUeRight;
						allocationMade = true;
						availableRBs--;
						users->at(newUeRight)->m_listOfAllocatedRBs.push_back(
								right);
						right++;
						std::cout << "allocated right to " << newUeRight
								<< std::endl;
						printf(
								"Scheduled User = %d  Required RB's = %d Allocated RB's= %d\n",
								newUeRight, requiredPRBs[newUeRight],
								users->at(newUeRight)->m_listOfAllocatedRBs.size());
					}			//End if verify right

					//}			//end right allocation
					if (users->at(newUeRight)->m_listOfAllocatedRBs.size()
							== requiredPRBs[newUeRight]) {
						allocatedUser[newUeRight] = true;
						std::cout << "finish allocation for this user"
								<< std::endl;
					}
					if ((right == nbOfRBs )&&( newUeRight !=selectedUser)){
						allocatedUser[newUeRight] = true;
					}

					ContinueLeft = true;

				}

				//Left Allocation
				for (int i = 1;
						i < requiredPRBs[newUeLeft] && (left >= 0)
								&& ContinueLeft && availableRBs > 0
								&& (!allocatedUser[newUeLeft]) && allocationMade
								&& (VerifyLeft) && (!Allocated[left]); i++) {

					//Allocate PRB at left to the user

					std::cout << "Continue Left" << std::endl;
					//HB
					//Verify if selected User has the best metric at left

					for (int j = 0; j < users->size(); j++) {
						if ((j != lastUeLeft) && !allocatedUser[j]
								&& (metrics[left][newUeLeft] < metrics[left][j])) {

							VerifyLeft = false;
							newUeLeft = j;

						}
					}
					if (VerifyLeft) {
						Allocated[left] = true;
						users->at(newUeLeft)->m_listOfAllocatedRBs.push_back(
								left);
						MAllocation[left] = newUeLeft;
						left--;
						allocationMade = true;
						availableRBs--;
						std::cout << "allocated left" << std::endl;
						printf(
								"Scheduled User = %d  Required RB's = %d Allocated RB's= %d\n",
								newUeLeft, requiredPRBs[newUeLeft],
								users->at(newUeLeft)->m_listOfAllocatedRBs.size());
					}
					//the adjacent RB don't correspond to selected User so RB will be allocated to the new UE
					else {
						Allocated[left] = true;
						allocatedUser[lastUeLeft] = true;
						lastUeLeft = newUeLeft;
						VerifyLeft = true;
						MAllocation[left] = newUeLeft;
						allocationMade = true;
						users->at(newUeLeft)->m_listOfAllocatedRBs.push_back(
								left);
						availableRBs--;
						left--;
						std::cout << "allocated left to " << newUeLeft
								<< std::endl;
						printf(
								"Scheduled User = %d  Required RB's = %d Allocated RB's= %d\n",
								newUeLeft, requiredPRBs[newUeLeft],
								users->at(newUeLeft)->m_listOfAllocatedRBs.size());
					}
					if (users->at(newUeLeft)->m_listOfAllocatedRBs.size()
							== requiredPRBs[newUeLeft]) {
						allocatedUser[newUeLeft] = true;
						std::cout << "finish allocation for this user"
								<< std::endl;
					}
					if(left == 0)
						allocatedUser[newUeLeft] = true;
					ContinueRight = true;
					//end Left allocation

				}		//end for
				if (left == 0)
					ContinueLeft = false;
				if (right == nbOfRBs)
					ContinueRight = false;

			}		//end While

		}  //end if
		else { // nothing to do exit the allocation loop
			break;
		}
		std::cout << "available RBs " << availableRBs << std::endl;
	}  //end While
	for (int i = 0; i < nbOfRBs; i++) {
		std::cout << "Mallocation[" << i << "] =" << MAllocation[i] << std::endl;
	}
} //end RB Allocation
