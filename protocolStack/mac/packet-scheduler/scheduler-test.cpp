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

#include "scheduler-test.h"
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
#include <vector>

#define SCHEDULER_DEBUG

SchedulerTest::SchedulerTest() {
	SetMacEntity(0);
	CreateUsersToSchedule();
}

SchedulerTest::~SchedulerTest() {
	Destroy();
}

double SchedulerTest::ComputeSchedulingMetric(RadioBearer *bearer,
		double spectralEfficiency, int subChannel) {
	double metric;
	return metric;
}

double SchedulerTest::ComputeSchedulingMetric(UserToSchedule* user,
		int subchannel) {
	double metric;

	int channelCondition = user->m_channelContition.at(subchannel);
	double spectralEfficiency = GetMacEntity()->GetAmcModule()->GetSinrFromCQI(
			channelCondition);

	metric = spectralEfficiency * 180000;

	return metric;
}

void SchedulerTest::RBsAllocation() {

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
	//	int requiredPRBs[users->size()];
		bool allocatedUser[users->size()];
		int MAllocation[nbOfRBs];
		int adjacentLeftUser;
		int adjacentRightUser;
		bool ContinueRight;
		bool ContinueLeft;
		int nbrOfScheduledUsers;
		std::vector<int> bestMetrics;
		std::vector<int> bestRBs;
		int nbRB[nbOfRBs];
		int nbRBRight[nbOfRBs];
		int nbRBLeft[nbOfRBs];
		int selectedI; //to identify the selected PRB
		//end HB
		//Some initialization
		nbrOfScheduledUsers = 0;
		availableRBs = nbOfRBs;
		unallocatedUsers = users->size();
		for (int i = 0; i < nbOfRBs; i++) {
			Allocated[i] = false;
			MAllocation[i] = -1;
		}

		for (int i = 0; i < users->size(); i++) {
			allocatedUser[i] = false;
		}
	//HB
	for (int i = 0; i < users->size(); i++) {
		allocatedUser[i] = false;
	}
	int requiredPRBs[users->size()]={5,5,7,7};
int I = 0;
int metric[nbOfRBs* users->size()]={2737,3353,1560,3353,1560,4224,3353,3353,2037,2737,1560,3837,2737,3353,2563,
			3353,2037,3353,3837,2737,2737,3353,2737,3353,2737,2037,3353,2737,3353,3353,
				3353,3353,2037,3353,1560,3353,3353,3353,2563,3837,3353,2563,3353,3353,2737,
			3353,3353,406,3837,2737,3353,2737,2737,3353,2737,3837,2737,2037,3353,406,
				-833,851,1560,851,1560,851,1560,1355,406,1560,406,851,406,406,851
};
	//create a matrix of flow metrics
	for (int j = 0; j < users->size(); j++){
		for (int i = 0; i < nbOfRBs; i++)
		 {
			metrics[i][j] = metric[I];
			I++;
		}
	}

	//create number of required PRB's per scheduled users
	for (int j = 0; j < users->size(); j++) {
		scheduledUser = users->at(j);
#ifdef SCHEDULER_DEBUG
		cout << "\n" << "User " << j << std::endl; // << "CQI Vector";
#endif
		std::cout << "CQI " << "\n";
		std::vector<double> sinrs;
		for (std::vector<int>::iterator c =
				scheduledUser->m_channelContition.begin();
				c != scheduledUser->m_channelContition.end(); c++) {
			std::cout << *c << " ";
			sinrs.push_back(GetMacEntity()->GetAmcModule()->GetSinrFromCQI(*c));
		}
		std::cout << "\n";
		double effectiveSinr = GetEesmEffectiveSinr(sinrs);

		int mcs = GetMacEntity()->GetAmcModule()->GetMCSFromCQI(
				GetMacEntity()->GetAmcModule()->GetCQIFromSinr(effectiveSinr));
		scheduledUser->m_selectedMCS = mcs;
		/*requiredPRBs[j] = (floor)(
				scheduledUser->m_dataToTransmit
						/ (GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(mcs,
								1) / 8));*/
#ifdef SCHEDULER_DEBUG
		cout << " EffSINR = " << effectiveSinr << "  MCS = " << mcs
				<< " Required RBs " << requiredPRBs[j] << " data to transmit "
				<< scheduledUser->m_dataToTransmit << " TB size "
				<< GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(mcs, 1)
				<< "\n";
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
			printf("%3d  ", (int) (metrics[jj][ii] ));
		}
		std::cout << std::endl;
	}
#endif

	//RBs allocation
		while (availableRBs > 0 && unallocatedUsers > 0) {
			//first step: find the best user-RB combinaision
			selectedPRB = -1;
			selectedUser = -1;
			ContinueRight = true;
			ContinueLeft = true;
			bestMetric = (double) (-(1 << 30));

			//Search the user with best metric
			for (int i = 0; i < nbOfRBs; i++) {
				if (!Allocated[i]) { // check only unallocated PRB's
					for (int j = 0; j < users->size(); j++) {
						if (users->at(j)->m_listOfAllocatedRBs.size() == 0
								&& requiredPRBs[j] > 0) //only unallocated users requesting some RB's
							if (bestMetric < metrics[i][j]) {
								selectedUser = j;
								bestMetric = metrics[i][j];

							}
					}

				}
			}
	#ifdef SCHEDULER_DEBUG
			std::cout << " selected User " << selectedUser << std::endl;
	#endif
			//Find the 10 best Metrics for selected Ue

			//initialization
			if (selectedUser != -1) {
				bestMetrics.clear();
				bestRBs.clear();
				for (int i = 0; i < nbOfRBs; i++) {
					if (!Allocated[i]) {
						bestMetrics.push_back(metrics[i][selectedUser]);
						bestRBs.push_back(i);
					}
				}
	//sort the bestMetrics
				for (int j = 0; j < bestMetrics.size(); j++) {
					for (int i = 0; i < bestMetrics.size() - 1; i++) {
						if (bestMetrics[i] < bestMetrics[i + 1]) {
							int a, b;
							a = bestMetrics[i];
							bestMetrics[i] = bestMetrics[i + 1];
							bestMetrics[i + 1] = a;
							b = bestRBs[i];
							bestRBs[i] = bestRBs[i + 1];
							bestRBs[i + 1] = b;

						}
					}
				}
				/*//affichage
				 std::cout << "bestMetrics\n " ;
				 for (int j = 0; j < bestMetrics.size(); j++)
				 std::cout << bestMetrics[j]<< std::endl;*/

				//find for each best metric the maximum RBs that can be allocated to selectedUE
				for (int i = 0; i < bestMetrics.size(); i++) {
					right = bestRBs[i] + 1;
					left = bestRBs[i] - 1;
					nbRBRight[i] = 0;
					nbRBLeft[i] = 0;
					if (left >= 0 && Allocated[left] && right < nbOfRBs
							&& Allocated[right]) {
						std::cout << "nothing is available" << std::endl;
						break; // nothing is available, since we need to have contiguous allocation
					}
					for (int k = right; k < nbOfRBs; k++) {
						if (!Allocated[k]) {
							for (int j = 0; j < users->size(); j++) {
								if ((j != selectedUser) && !allocatedUser[j]
										&& (metrics[k][selectedUser] < metrics[k][j])) {
									break;
								}
								nbRBRight[i]++;
							}
						}
					} //end right
					for (int k = left; k >= 0; k--) {
						if (!Allocated[k]) {
							for (int j = 0; j < users->size(); j++) {
								if ((j != selectedUser) && !allocatedUser[j]
										&& (metrics[k][selectedUser] < metrics[k][j])) {
									break;
								}
								nbRBLeft[i]++;
							}
						}
					} //end left
				} //end calculation of possible allocatedRB for each CQI
				  //Choisir le CQI qui permet d'avoir le max allocated RB
				for (int i = 0; i < bestMetrics.size(); i++) {
					nbRB[i] = 1 + nbRBLeft[i] + nbRBRight[i];
					if (nbRB[i] >= requiredPRBs[selectedUser]) {
						selectedPRB = bestRBs[i];
						selectedI = i;
						break;
					} else {
						for (int j = 0; j < bestMetrics.size(); j++) {
							if (nbRB[j] > nbRB[i])
								break;
						}
						selectedPRB = bestRBs[i];
						selectedI = i;
						break;
					}

				}
	//			std::cout << "selected PRB" << selectedPRB << std::endl;
				//fin choix CQI
				//Allocation as RME scheduler
				// not necessary to verify ! allocated and if it correspond to UE it is already done
				// for to RBRight and RBLeft

				scheduledUser = users->at(selectedUser);
				scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);
				Allocated[selectedPRB] = true;
				left = selectedPRB - 1;
				right = selectedPRB + 1;
				availableRBs--;
				MAllocation[selectedPRB] = selectedUser;
				while ((scheduledUser->m_listOfAllocatedRBs.size()
						!= requiredPRBs[selectedUser])
						&& (ContinueRight || ContinueLeft) && availableRBs > 0) {

					if (right < (selectedPRB + nbRBRight[selectedI])) {
						Allocated[right] = true;
						users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
								right);
						MAllocation[right] = selectedUser;
						right++;
						availableRBs--;
					} else
						ContinueRight = false;
					if (left > (selectedPRB - nbRBLeft[selectedI])) {
						Allocated[left] = true;
						MAllocation[left] = selectedUser;
						users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
								left);
						left--;
						availableRBs--;
					} else
						ContinueLeft = false;
				}
				unallocatedUsers--;
			}		//end if selectedUser !=-1
			else
				//no more users to allocate
				break;
		}	//end while

		//not all RBs are allocated
		int i = 0;

		while (i < nbOfRBs && availableRBs < (nbOfRBs - 1)) {
			if (!Allocated[i]) {
	#ifdef SCHEDULER_DEBUG
				std::cout << "RB " << i << " is not yet allocated" << std::endl;
	#endif
				left = i - 1;
				right = i + 1;
				while (right < nbOfRBs && (!Allocated[right])) {
					right++;
				}


				if (left == -1 && MAllocation[right] == -1) {
					std::cout << "RB cn't be allocated" << std::endl;
				}
				//case i =0  or the left RB can't be allocated
				else if (left == -1) {
					selectedUser = MAllocation[right];
					int j = right - 1;
					while (j >= 0) {
						if (users->at(selectedUser)->m_listOfAllocatedRBs.size()
								!= requiredPRBs[selectedUser]) {
							Allocated[j] = true;
							MAllocation[j] = selectedUser;
							users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
									j);
							std::cout << "allocated RB " << j << " to "
									<< selectedUser << std::endl;
							j--;
							i++;
							availableRBs--;
						} else
							break; //can't allocate RB
					}
				} //end cas i =0
				  //case not allocated RBs are in the end of RBs
				else if (right == nbOfRBs) {
					selectedUser = MAllocation[left];
					int j = i;
					while (j < nbOfRBs) {
						if (users->at(selectedUser)->m_listOfAllocatedRBs.size()
								!= requiredPRBs[selectedUser]) {
							Allocated[j] = true;
							MAllocation[j] = selectedUser;
							users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
									j);
							std::cout << "allocated RB " << j << " to "
										<< selectedUser << std::endl;
							j++;
							i++;
							availableRBs--;
						} else {
							std::cout << "can't allocated the rest of RBs" << std::endl;
							j = nbOfRBs;
							i = nbOfRBs;
							break;

						}
						//can't allocated RB
					}
				} //end case not allocated Rbs are in the end
				  //case not allocated Rbs are between allocated UEs
				else {
					adjacentRightUser = MAllocation[right];
					adjacentLeftUser = MAllocation[left];
					/*M[left]>M[right]
					 * * if adjacentLeft didn't reach its maximum so RB is allocated to adjacentLeft
					 * * else
					 * *** if adjacent Right didn't reach its maximum so RB is allocated to adjacent Right
					 * ***else RB is not allocated
					 *
					 */
					if (MAllocation[left] != -1) {
						if (metrics[i][adjacentLeftUser]
								>= metrics[i][adjacentRightUser]) {
							if (users->at(adjacentLeftUser)->m_listOfAllocatedRBs.size()
									< requiredPRBs[adjacentLeftUser]) {
								selectedUser = adjacentLeftUser;
								Allocated[i] = true;
								MAllocation[i] = selectedUser;
								users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
										i);
								std::cout << "allocated RB " << i << " to "
																		<< selectedUser << std::endl;
								availableRBs--;
							}

							else if ((users->at(adjacentRightUser)->m_listOfAllocatedRBs.size()
									+ right - i)
									<= requiredPRBs[adjacentLeftUser]) {
								std::cout << "last to UE" << adjacentRightUser
																		<< " RB "
																		<< users->at(adjacentRightUser)->m_listOfAllocatedRBs.size()
																				+ right - i << std::endl;
								selectedUser = adjacentRightUser;
								Allocated[i] = true;
								MAllocation[i] = selectedUser;
								users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
										i);
								std::cout << "allocated RB " << i << " to "
																		<< selectedUser << std::endl;
								availableRBs--;
							}
						}

						/*M[left]<M[right]
						 * * if adjacentRight didn't reach its maximum so RB is allocated to adjacentRight
						 * * else
						 * *** if adjacent Leftt didn't reach its maximum so RB is allocated to adjacent Left
						 * ***else RB is not allocated
						 *
						 */
						else {
							if ((users->at(adjacentRightUser)->m_listOfAllocatedRBs.size()
									+ right - i)
									<= requiredPRBs[adjacentLeftUser]) {
								std::cout << "last to UE" << adjacentRightUser
																		<< " RB "
																		<< users->at(adjacentRightUser)->m_listOfAllocatedRBs.size()
																				+ right - i << std::endl;
								selectedUser = adjacentRightUser;
								Allocated[i] = true;
								MAllocation[i] = selectedUser;
								users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
										i);
								std::cout << "allocated RB " << i << " to "
																		<< selectedUser << std::endl;
								availableRBs--;
							}

							else if (users->at(adjacentLeftUser)->m_listOfAllocatedRBs.size()
									< requiredPRBs[adjacentLeftUser]) {
								selectedUser = adjacentLeftUser;
								Allocated[i] = true;
								MAllocation[i] = selectedUser;
								users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
										i);
								std::cout << "allocated RB " << i << " to "
																		<< selectedUser << std::endl;
								availableRBs--;
							}
						}
					}

				}
			}
			i++;
		}
		//Affichage
	#ifdef SCHEDULER_DEBUG
		for (int i = 0; i < nbOfRBs; i++) {
			std::cout << "Mallocation[" << i << "] =" << MAllocation[i]
			<< std::endl;
		}
	#endif
		for (int j = 0; j < users->size(); j++) {
			UserToSchedule* scheduledUser;
			scheduledUser = users->at(j);
			scheduledUser->m_transmittedData =
					GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(
							scheduledUser->m_selectedMCS,
							scheduledUser->m_listOfAllocatedRBs.size()) / 8;
	#ifdef SCHEDULER_DEBUG
			printf(
					"Scheduled User = %d mcs = %d Required RB's = %d Allocated RB's= %d\n",
					scheduledUser->m_userToSchedule->GetIDNetworkNode(),
					scheduledUser->m_selectedMCS, requiredPRBs[j],
					scheduledUser->m_listOfAllocatedRBs.size());
			for (int i = 0; i < scheduledUser->m_listOfAllocatedRBs.size(); i++)
			printf("%d ", scheduledUser->m_listOfAllocatedRBs.at(i));

	#endif
			//Calculate power
			if (scheduledUser->m_listOfAllocatedRBs.size() == 0)
				scheduledUser->m_power += 0;
			else
				scheduledUser->m_power += CalculatePower(
						users->at(j)->m_listOfAllocatedRBs.size(), scheduledUser);
	#ifdef SCHEDULER_DEBUG
			std::cout << "power["
			<< scheduledUser->m_userToSchedule->GetIDNetworkNode() << "]= "
			<< scheduledUser->m_power << std::endl;
	//RBs /user/TTI
			std::cout << "My Scheduler NRbs of "
			<< scheduledUser->m_userToSchedule->GetIDNetworkNode() << " = "
			<< scheduledUser->m_listOfAllocatedRBs.size() << std::endl;
			printf("\n------------------\n");
	#endif
			//number of scheduled users per TTI
			if (scheduledUser->m_listOfAllocatedRBs.size() > 0)
				nbrOfScheduledUsers++;

		}
std::cout << "number of scheduled users" << std::endl;
	}
