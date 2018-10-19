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

#define SCHEDULER_DEBUG

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
	#ifdef SCHEDULER_DEBUG
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
		//HB
		bool ContinueRight;
		bool ContinueLeft;
		bool allocatedUser[users->size()];
		int MAllocation[nbOfRBs];
		int adjacentLeftUser;
		int adjacentRightUser;

		//end HB
		//Some initialization
		availableRBs = nbOfRBs;
		unallocatedUsers = users->size();
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
				//cout << *c <<" ";
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

		//RBs allocation

		while (availableRBs > 0 && unallocatedUsers >0) //
		{
			// First step: find the best user-RB combo
			selectedPRB = -1;
			selectedUser = -1;
			ContinueRight = true;
			ContinueLeft = true;
			bestMetric = (double) (-(1 << 30));
			std::cout << "now= " << Simulator::Init()->Now() << std::endl;
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
			std::cout << "**bestMetric  = " << bestMetric / 1000.0
					<< " selected prb " << selectedPRB << " selected User "
					<< selectedUser << std::endl;

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
							std::cout << "right allocation" << std::endl;
						}
					} else
						ContinueRight = false;
					//Start Left allocation
					if (left >= 0 && !Allocated[left]) {
						for (int j = 0; j < users->size(); j++) {

							if ((j != selectedUser) && !allocatedUser[j]
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
							std::cout << "left allocation" << std::endl;
						}
					} else
						ContinueLeft = false;

				}
				allocatedUser[selectedUser] = true;
				unallocatedUsers--;
				std::cout << "last RB left = " << left + 1 << " RB right = "
						<< right - 1 << std::endl;

			} //end if
			std::cout << "unallocatedUsers " << unallocatedUsers << std::endl;
		} //end While

		/*while (availableRBs > 0 ){*/
	//Case unallocated RBs
		std::cout << "all users are allocated and still availabale " << availableRBs
				<< std::endl;
		int i = 0;
		while( i < nbOfRBs) {
			if (!Allocated[i]) {
				left = i - 1;
				right = i + 1;
				while (right < nbOfRBs && (!Allocated[right])) {
					right++;
				}

				//case i =0
				if (left == -1) {
					selectedUser = MAllocation[right];
					int j = right - 1;
					while (j >= 0) {
						if (users->at(selectedUser)->m_listOfAllocatedRBs.size()
								!= requiredPRBs[selectedUser]) {
							Allocated[j] = true;
							MAllocation[j] = selectedUser;
							users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
									j);
							std::cout << "allocated RB " << j << " to " << selectedUser << std::endl;
							j--;
							availableRBs--;
						} else
							break; //can't allocate RB
					}
				} //end cas i =0
				  //case not allocated RBs are in the end of RBs
				else if (right == nbOfRBs) {
					selectedUser = MAllocation[i - 1];
					int j = i;
					while (j < nbOfRBs) {
						if (users->at(selectedUser)->m_listOfAllocatedRBs.size()
								!= requiredPRBs[selectedUser]) {
							Allocated[j] = true;
							MAllocation[j] = selectedUser;
							users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
									j);
							std::cout << "allocated RB " << j << " to " << selectedUser << std::endl;
							j++;
							availableRBs--;
						} else
							break;			//can't allocated RB
					}
				}				  //end case not allocated Rbs are in the end
								  //case not allocated Rbs are between allocated UEs
				else {
					adjacentRightUser = MAllocation[right];
					adjacentLeftUser = MAllocation[left];
					int j = i;
					while(j < nbOfRBs)
					{
					if ((metrics[j][adjacentLeftUser]
							>= metrics[j][adjacentRightUser])
							&& (users->at(adjacentLeftUser)->m_listOfAllocatedRBs.size()
									!= requiredPRBs[adjacentLeftUser]))	//Left is better than right and still need RBs
							{
						selectedUser = adjacentLeftUser;
					} else if ((users->at(adjacentRightUser)->m_listOfAllocatedRBs.size()
							+ right - i - 1) <= requiredPRBs[adjacentLeftUser])
						selectedUser = adjacentRightUser;

					Allocated[j] = true;
					MAllocation[j] = selectedUser;
					users->at(selectedUser)->m_listOfAllocatedRBs.push_back(j);
					std::cout << "allocated RB " << j << " to " << selectedUser << std::endl;
					j++;
					availableRBs--;
				}
				}
				break;
			}
			i++;
		}

	}
