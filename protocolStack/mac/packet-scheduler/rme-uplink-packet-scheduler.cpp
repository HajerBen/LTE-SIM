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

#include "rme-uplink-packet-scheduler.h"
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
RecursiveMaximumExpansion::RecursiveMaximumExpansion() {
	SetMacEntity(0);
	CreateUsersToSchedule();
}

RecursiveMaximumExpansion::~RecursiveMaximumExpansion() {
	Destroy();
}

double RecursiveMaximumExpansion::ComputeSchedulingMetric(RadioBearer *bearer,
		double spectralEfficiency, int subChannel) {
	double metric;
	return metric;
}

double RecursiveMaximumExpansion::ComputeSchedulingMetric(UserToSchedule* user,
		int subchannel) {
	double metric;

	int channelCondition = user->m_channelContition.at(subchannel);
	double spectralEfficiency = GetMacEntity()->GetAmcModule()->GetSinrFromCQI(
			channelCondition);

	metric = spectralEfficiency * 180000;

	return metric;
}

void RecursiveMaximumExpansion::RBsAllocation() {
	/* This is an implementation of an algorithm based on  Recursive Maximum Expansion algorithm (RME) reported in
	 * L. Temiño, G. Berardinelli, S. Frattasi,  and P.E. Mogensen,
	 * "Channel-aware scheduling algorithms for SC-FDMA in LTE uplink",  in Proc. PIMRC 2008
	 *
	 */
#ifdef Allocation
	std::cout << " ---- UL RBs Allocation(rme)";
#endif

	UsersToSchedule *users = GetUsersToSchedule();
	UserToSchedule* scheduledUser;
	int nbOfRBs =
			GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();

	int availableRBs;     // No of RB's not allocated
	int unallocatedUsers; // No of users who remain unallocated
	int selectedUser;     // user to be selected for allocation
	int selectedPRB;      // PRB to be selected for allocation
	double bestMetric;    // best metric to identify user/RB combination
	int left, right;      // index of left and left PRB's to check
	bool Allocated[nbOfRBs];
	double metrics[nbOfRBs][users->size()];
	int requiredPRBs[users->size()];
	bool ContinueRight;
	bool ContinueLeft;
	bool allocatedUser[users->size()];
	int MAllocation[nbOfRBs];
	int adjacentLeftUser;
	int adjacentRightUser;
	int nbrOfScheduledUsers;

	//Some initialization
	availableRBs = nbOfRBs;
	unallocatedUsers = users->size();
	nbrOfScheduledUsers = 0;
	for (int i = 0; i < nbOfRBs; i++) {
		Allocated[i] = false;
		MAllocation[i] = -1;
	}
	//HB
	for (int i = 0; i < users->size(); i++) {
		allocatedUser[i] = false;
	}
	//end HB
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

#ifdef Allocation
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
	std::cout << "now= " << Simulator::Init()->Now() << std::endl;
#endif

	//RBs allocation

	while (availableRBs > 0 && unallocatedUsers > 0) //
	{
		// First step: find the best user-RB combo
		selectedPRB = -1;
		selectedUser = -1;
		ContinueRight = true;
		ContinueLeft = true;
		bestMetric = (double) (-(1 << 30));

		//Search for the best metric
		for (int i = 0; i < nbOfRBs; i++) {
			if (!Allocated[i]) { // check only unallocated PRB's
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
#ifdef Allocation
		std::cout << "**bestMetric  = " << bestMetric / 1000.0
		<< " selected prb " << selectedPRB << " selected User "
		<< selectedUser << std::endl;
#endif
		if (selectedUser != -1) {
			scheduledUser = users->at(selectedUser);
			scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);
			Allocated[selectedPRB] = true;
			left = selectedPRB - 1;
			right = selectedPRB + 1;
			availableRBs--;
			MAllocation[selectedPRB] = selectedUser;

			if (left >= 0 && Allocated[left] && right < nbOfRBs
					&& Allocated[right])
				break; // nothing is available, since we need to have contiguous allocation

			while ((scheduledUser->m_listOfAllocatedRBs.size()
					!= requiredPRBs[selectedUser])
					&& (ContinueRight || ContinueLeft) && availableRBs > 0) {
				//start right allocation
				if (right < nbOfRBs && !Allocated[right]) {
					//Verify if the best metric at right doesn't correspond to UE
					for (int j = 0; j < users->size(); j++) {

						if ((j != selectedUser) && !allocatedUser[j]
								&& requiredPRBs[j] > 0
								&& (metrics[right][selectedUser]
										< metrics[right][j])) {
							ContinueRight = false;
							break;
						}
					}
					if (ContinueRight) {
						Allocated[right] = true;
						users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
								right);
						MAllocation[right] = selectedUser;
						right++;
						availableRBs--;
						//std::cout << "right allocation" << std::endl;
					}
				} else
					ContinueRight = false;
				//Start Left allocation
				if (left >= 0 && !Allocated[left]) {
					for (int j = 0; j < users->size(); j++) {
						if ((j != selectedUser) && !allocatedUser[j]
								&& requiredPRBs[j] > 0
								&& (metrics[left][selectedUser]
										< metrics[left][j])) {
							ContinueLeft = false;
							break;
						}
					}
					if (ContinueLeft) {
						Allocated[left] = true;
						MAllocation[left] = selectedUser;
						users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
								left);
						left--;
						availableRBs--;
					}
				} else
					ContinueLeft = false;
			}
			allocatedUser[selectedUser] = true;
			unallocatedUsers--;
		} //end if
		else
			//no more users to allocate
			break;
#ifdef Allocation
		std::cout << "unallocatedUsers " << unallocatedUsers << std::endl;
#endif
	} //end While

	/*while (availableRBs > 0 ){*/
	//Case unallocated RBs
#ifdef Allocation
	std::cout << "all users are allocated and still availabale " << availableRBs
	<< std::endl;
#endif
	int i = 0;

	while (i < nbOfRBs && availableRBs < (nbOfRBs - 1) && availableRBs > 0) {
		if (!Allocated[i]) {
#ifdef Allocation
			std::cout << "RB " << i << " is not yet allocated" << std::endl;
#endif
			left = i - 1;
			right = i + 1;
			while (right < nbOfRBs && (!Allocated[right])) {
				right++;
			}

			//case i =0
			if (left == -1 && MAllocation[right] == -1) {
				std::cout << "RB can't be allocated" << std::endl;
			} else if (left == -1) {
				selectedUser = MAllocation[right];

				for (int j = 0; j < right; j++) {

					Allocated[j] = true;
					MAllocation[j] = selectedUser;
					users->at(selectedUser)->m_listOfAllocatedRBs.push_back(j);
					i++;
					availableRBs--;
				}

			} //end cas i =0
			  //case not allocated RBs are in the end of RBs
			else if (right == nbOfRBs) {
				selectedUser = MAllocation[left];
				int j = i;
				for (int j = left + 1; j < nbOfRBs; j++) {

					Allocated[j] = true;
					MAllocation[j] = selectedUser;
					users->at(selectedUser)->m_listOfAllocatedRBs.push_back(j);
					i++;
					availableRBs--;
				}
				//can't allocated RB

			} //end case not allocated Rbs are in the end

			//case not allocated Rbs are between allocated UEs
			else {
				adjacentRightUser = MAllocation[right];
				adjacentLeftUser = MAllocation[left];
				bestMetric = metrics[i][adjacentLeftUser];
				for (int k = left + 1; k < right; k++) {
					for (int j = 0; j < users->size(); j++) {
						if ((bestMetric < metrics[k][j])
								&& ((j == adjacentLeftUser)
										|| (j == adjacentRightUser))) {
							selectedPRB = k;
							selectedUser = j;
							bestMetric = metrics[k][j];

						}
					}
				}
				right = selectedPRB + 1;
				left = selectedPRB - 1;
				ContinueRight = ContinueLeft = true;
				while ((ContinueRight || ContinueLeft) && availableRBs > 0) {
					//start right allocation
					if (right < nbOfRBs && !Allocated[right]) {
						//Verify if the best metric at right doesn't correspond to UE
						for (int j = 0; j < users->size(); j++) {

							if ((j != selectedUser) && !allocatedUser[j]
									&& requiredPRBs[j] > 0
									&& (metrics[right][selectedUser]
											< metrics[right][j])) {
								ContinueRight = false;
								break;
							}
						}
						if (ContinueRight) {
							Allocated[right] = true;
							users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
									right);
							MAllocation[right] = selectedUser;
							right++;
							availableRBs--;
							//std::cout << "right allocation" << std::endl;
						}
					} else
						ContinueRight = false;
					//Start Left allocation
					if (left >= 0 && !Allocated[left]) {
						for (int j = 0; j < users->size(); j++) {
							if ((j != selectedUser) && !allocatedUser[j]
									&& requiredPRBs[j] > 0
									&& (metrics[left][selectedUser]
											< metrics[left][j])) {
								ContinueLeft = false;
								break;
							}
						}
						if (ContinueLeft) {
							Allocated[left] = true;
							MAllocation[left] = selectedUser;
							users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
									left);
							left--;
							availableRBs--;
						}
					} else
						ContinueLeft = false;
				}
			}}
			i++;

	}

	//Affichage
	/*
	 #ifdef Allocation
	 for (int i = 0; i < nbOfRBs; i++) {
	 std::cout << "Mallocation[" << i << "] =" << MAllocation[i]
	 << std::endl;
	 }
	 #endif
	 */

//Calculate power
	for (int j = 0; j < users->size(); j++) {
		UserToSchedule* scheduledUser1;
		scheduledUser1 = users->at(j);
		scheduledUser1->m_transmittedData =
				GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(
						scheduledUser1->m_selectedMCS,
						scheduledUser1->m_listOfAllocatedRBs.size()) / 8;
		if (scheduledUser1->m_listOfAllocatedRBs.size() == 0)
			scheduledUser1->m_power += 0;
		else
			scheduledUser1->m_power += CalculatePower(
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
	std::cout << "number of scheduled users per TTI " << nbrOfScheduledUsers
			<< std::endl;
} //end RB Allocation
