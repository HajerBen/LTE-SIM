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
 * Author:  Hajer Ben Rekhissa
 */

#include "myScheduler.h"
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
#define Allocation
myScheduler::myScheduler() {
	SetMacEntity(0);
	CreateUsersToSchedule();
}

myScheduler::~myScheduler() {
	Destroy();
}

double myScheduler::ComputeSchedulingMetric(RadioBearer *bearer,
		double spectralEfficiency, int subChannel) {
	double metric;
	return metric;
}

double myScheduler::ComputeSchedulingMetric(UserToSchedule* user,
		int subchannel) {
	double metric;

	int channelCondition = user->m_channelContition.at(subchannel);
	double spectralEfficiency = GetMacEntity()->GetAmcModule()->GetSinrFromCQI(
			channelCondition);

	metric = spectralEfficiency;

	return metric;
}
double myScheduler::ComputeSchedulingMetric(UserToSchedule* user) {
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
	std::cout << "delay " << metric << std::endl;
	return metric;
}

//Sort CQI for each user
void myScheduler::ChannelSorted(int SizeOfChannelSorted,
		std::vector<int> &ChannelsSorted, std::vector<int> &bestRBs) {
	for (int j = 0; j < SizeOfChannelSorted; j++) {
		for (int i = 0; i < SizeOfChannelSorted - 1; i++) {
			if (ChannelsSorted[i] < ChannelsSorted[i + 1]) {
				int a, b;
				a = ChannelsSorted[i];
				ChannelsSorted[i] = ChannelsSorted[i + 1];
				ChannelsSorted[i + 1] = a;
				b = bestRBs[i];
				bestRBs[i] = bestRBs[i + 1];
				bestRBs[i + 1] = b;

			}
		}
	}
}
void myScheduler::RBsAllocation() {

#ifdef SCHEDULER_DEBUG
	std::cout << " ---- UL RBs Allocation(myScheduler)";
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
	double lowestDelay;	  //lowest delay to identify UE with lowest delay
	int left, right;      // index of left and left PRB's to check
	bool Allocated[nbOfRBs];
	double metrics[nbOfRBs][users->size()];
	int requiredPRBs[users->size()];
	bool allocatedUser[users->size()];
	int MAllocation[nbOfRBs];
	int adjacentLeftUser;
	int adjacentRightUser;
	bool ContinueRight;
	bool ContinueLeft;
	int nbrOfScheduledUsers;
	std::vector<int> ChannelsSorted;
	std::vector<int> bestRBs;
	int nbRB[nbOfRBs];
	int nbRBRight[nbOfRBs];
	int nbRBLeft[nbOfRBs];
	int selectedI; //to identify the selected PRB
	int delay_threshold;
	//end HB
	//Some initialization
	delay_threshold = 3;
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
//Create matrix of flow metrics:delay
	double delay[users->size()];
	for (int i = 0; i < users->size(); i++) {
		delay[i] = ComputeSchedulingMetric(users->at(i));
	}

	//create a matrix of channel quality
	for (int i = 0; i < nbOfRBs; i++) {
		for (int j = 0; j < users->size(); j++) {
			metrics[i][j] = ComputeSchedulingMetric(users->at(j), i);
		}
	}
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
//			std::cout << *c << " ";
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

#ifdef Allocation
	std::cout << " available RBs " << nbOfRBs << ", users " << users->size()
			<< std::endl;
	for (int ii = 0; ii < users->size(); ii++) {
		std::cout << "channel quality for user "
				<< users->at(ii)->m_userToSchedule->GetIDNetworkNode() << "\n";
		for (int jj = 0; jj < nbOfRBs; jj++) {
			//std::cout  << setw(3) << metrics[jj][ii]/1000 << " ";
			printf("%3d  ", (int) (metrics[jj][ii]));
		}
		std::cout << std::endl;
	}
	std::cout << "now= " << Simulator::Init()->Now() << std::endl;
#endif

	//RBs allocation
	while (availableRBs > 0 && unallocatedUsers > 0) {
		//first step: find the best user-RB combinaision
		selectedPRB = -1;
		selectedUser = -1;
		ContinueRight = true;
		ContinueLeft = true;
		bestMetric = (double) (-(1 << 30));
		lowestDelay = (double) ((1 << 30));
		//Search for the user with lowest delay
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

		//if lowest_delay <= delay_threshold => UE has the highest priority
		// Ue selected will start sending paquets from its highest CQI and ignore other users
		if (lowestDelay <= delay_threshold && (selectedUser != -1)) {
#ifdef Allocation
			std::cout << " selected User " << selectedUser << std::endl;
#endif
			for (int i = 0; i < nbOfRBs; i++) {
				if (!Allocated[i]) { // check only unallocated PRB's

					if (users->at(selectedUser)->m_listOfAllocatedRBs.size()
							== 0 && requiredPRBs[selectedUser] > 0) //only unallocated users requesting some RB's
						if (bestMetric < metrics[i][selectedUser]) {
							selectedPRB = i;
							bestMetric = metrics[i][selectedUser];
						}
				}
			}

			//start Allocation
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
					Allocated[right] = true;
					users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
							right);
					MAllocation[right] = selectedUser;
					right++;
					availableRBs--;
					ContinueRight = true;
					//std::cout << "right allocation" << std::endl;
				} else
					ContinueRight = false;

				//Start Left allocation
				if (left >= 0 && !Allocated[left]) {
					Allocated[left] = true;
					MAllocation[left] = selectedUser;
					users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
							left);
					left--;
					availableRBs--;
					ContinueLeft = true;
				} else
					ContinueLeft = false;
			}
			allocatedUser[selectedUser] = true;
			unallocatedUsers--;
		} //end if lowestDelay < delayThreshold

		//if there is no UE which reach the delayThreshold so the selected User is the Ue with best CQI
		else {

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
			std::cout << " selected User " << selectedUser << std::endl;
#endif
			//Find the 10 best Metrics for selected Ue

			//initialization
			if (selectedUser != -1) {
				ChannelsSorted.clear();
				bestRBs.clear();
				//initialization
				for (int i = 0; i < nbOfRBs; i++) {
					nbRBRight[i] = 0;
					nbRBLeft[i] = 0;
					nbRB[i] = 0;
				}

				for (int i = 0; i < nbOfRBs; i++) {
					if (!Allocated[i]) {
						ChannelsSorted.push_back(metrics[i][selectedUser]);
						bestRBs.push_back(i);
					}
				}
//sort the ChannelsSorted
				/*		for (int j = 0; j < ChannelsSorted.size(); j++) {
				 for (int i = 0; i < ChannelsSorted.size() - 1; i++) {
				 if (ChannelsSorted[i] < ChannelsSorted[i + 1]) {
				 int a, b;
				 a = ChannelsSorted[i];
				 ChannelsSorted[i] = ChannelsSorted[i + 1];
				 ChannelsSorted[i + 1] = a;
				 b = bestRBs[i];
				 bestRBs[i] = bestRBs[i + 1];
				 bestRBs[i + 1] = b;

				 }
				 }
				 }*/
				ChannelSorted(ChannelsSorted.size(), ChannelsSorted, bestRBs);
				//affichage
#ifdef ChannelSorted
				std::cout << "ChannelsSorted\n ";
				for (int j = 0; j < ChannelsSorted.size(); j++)
				std::cout << ChannelsSorted[j] << std::endl;
#endif

				//find for each best metric the maximum RBs that can be allocated to selectedUE
				for (int i = 0; i < ChannelsSorted.size(); i++) {
					bool Continue = true;
					//verify if the bestRBs[i] can't be allocated to an other UE
					for (int j = 0; j < users->size(); j++) {
						if ((j != selectedUser) && !allocatedUser[j]
								&& requiredPRBs[j] > 0
								&& (metrics[bestRBs[i]][selectedUser]
										< metrics[bestRBs[i]][j])) {
							nbRB[i] = 0;
							Continue = false;
							break;
						}
					}

					if (Continue) {
						right = bestRBs[i] + 1;
						left = bestRBs[i] - 1;
						nbRBRight[i] = 0;
						nbRBLeft[i] = 0;
						ContinueRight = ContinueLeft = true;
						if (left >= 0 && Allocated[left] && right < nbOfRBs
								&& Allocated[right]) {
							break; // nothing is available, since we need to have contiguous allocation
						}
						for (int k = right; k < nbOfRBs && ContinueRight; k++) {
							if (!Allocated[k]) {
								for (int j = 0; j < users->size(); j++) {
									if ((j != selectedUser) && !allocatedUser[j]
											&& requiredPRBs[j] > 0
											&& (metrics[k][selectedUser]
													< metrics[k][j])) {
										ContinueRight = false;
										break;
									}
								}
								if (ContinueRight)
									nbRBRight[i]++;
							} else
								break;
						} //end right
						for (int k = left; k >= 0 && ContinueLeft; k--) {
							if (!Allocated[k]) {
								for (int j = 0; j < users->size(); j++) {
									if ((j != selectedUser) && !allocatedUser[j]
											&& requiredPRBs[j] > 0
											&& (metrics[k][selectedUser]
													< metrics[k][j])) {
										ContinueLeft = false;
										break;
									}
								}
								if (ContinueLeft)
									nbRBLeft[i]++;
							} else
								break;
						} //end left
						nbRB[i] = 1 + nbRBLeft[i] + nbRBRight[i];
					}
				} //end calculation of possible allocatedRB for each CQI
				bool Verify;
				//Choisir le CQI qui permet d'avoir le max allocated RB
				for (int i = 0; i < ChannelsSorted.size(); i++) {
					Verify = true;
					if (nbRB[i] >= requiredPRBs[selectedUser]) {
						Verify = true;
					} else {
						for (int j = i; j < ChannelsSorted.size(); j++) {
							if (nbRB[j] > nbRB[i]) {
								Verify = false;
								break;
							}
						}
					}
					if (Verify) {
						selectedPRB = bestRBs[i];
						selectedI = i;
						break;
					}

				}
#ifdef Allocation
				std::cout << "selected PRB" << selectedPRB << std::endl;
#endif
				//fin choix CQI
				//Allocation as RME scheduler
				// not necessary to verify ! allocated and if it correspond to UE it is already done
				// for to RBRight and RBLeft

				scheduledUser = users->at(selectedUser);
				scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);
				allocatedUser[selectedUser] = true;
				Allocated[selectedPRB] = true;
				left = selectedPRB - 1;
				right = selectedPRB + 1;
				ContinueRight = ContinueLeft = true;
				availableRBs--;
				MAllocation[selectedPRB] = selectedUser;
				while ((scheduledUser->m_listOfAllocatedRBs.size()
						< requiredPRBs[selectedUser])
						&& (ContinueRight || ContinueLeft) && availableRBs > 0) {

					if ((right <= (selectedPRB + nbRBRight[selectedI]))
							&& (scheduledUser->m_listOfAllocatedRBs.size()
									< requiredPRBs[selectedUser])) {
						Allocated[right] = true;
						users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
								right);
						MAllocation[right] = selectedUser;
						right++;
						availableRBs--;
					} else
						ContinueRight = false;
					if ((left >= (selectedPRB - nbRBLeft[selectedI]))
							&& (scheduledUser->m_listOfAllocatedRBs.size()
									< requiredPRBs[selectedUser])) {
						Allocated[left] = true;
						MAllocation[left] = selectedUser;
						users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
								left);
						left--;
						availableRBs--;
					} else
						ContinueLeft = false;
				}
				for (int i = 0; i < scheduledUser->m_listOfAllocatedRBs.size();
						i++)
					printf("%d ", scheduledUser->m_listOfAllocatedRBs.at(i));

				printf("\n------------------\n");

				unallocatedUsers--;

			}		//end if selectedUser !=-1
			else
				//no more users to allocate
				break;
		}
	}
//end while

//not all RBs are allocated
	int i = 0;

	while (i < nbOfRBs && availableRBs < (nbOfRBs - 1)) {
		if (!Allocated[i]) {
#ifdef Allocation
			std::cout << "RB " << i << " is not yet allocated" << std::endl;
#endif
			left = i - 1;
			right = i + 1;
			while (right < nbOfRBs && (!Allocated[right])) {
				right++;
			}

			if (left == -1 && MAllocation[right] == -1) {
#ifdef Allocation
				std::cout << "RB can't be allocated" << std::endl;
#endif
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
#ifdef Allocation
						std::cout << "allocated RB " << i << " to "
								<< selectedUser << std::endl;
#endif
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
#ifdef Allocation
						std::cout << "allocated RB " << i << " to "
								<< selectedUser << std::endl;
#endif
						j++;
						i++;
						availableRBs--;
					} else {
#ifdef Allocation
						std::cout << "can't allocated the rest of RBs"
								<< std::endl;
#endif
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
#ifdef Allocation
							std::cout << "allocated RB " << i << " to "
									<< selectedUser << std::endl;
#endif
							availableRBs--;
						} else if ((users->at(adjacentRightUser)->m_listOfAllocatedRBs.size()
								+ right - i)
								<= requiredPRBs[adjacentRightUser]) {
							std::cout << " allocated "
									<< users->at(adjacentRightUser)->m_listOfAllocatedRBs.size()
											+ right - i << " required "
									<< requiredPRBs[adjacentRightUser]
									<< std::endl;
							selectedUser = adjacentRightUser;
							Allocated[i] = true;
							MAllocation[i] = selectedUser;
							users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
									i);
#ifdef Allocation
							std::cout << "allocated RB " << i << " to "
									<< selectedUser << std::endl;
#endif
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
							selectedUser = adjacentRightUser;
							Allocated[i] = true;
							MAllocation[i] = selectedUser;
							users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
									i);
#ifdef Allocation
							std::cout << "allocated RB " << i << " to "
									<< selectedUser << std::endl;
#endif
							availableRBs--;
						}

						else if (users->at(adjacentLeftUser)->m_listOfAllocatedRBs.size()
								< requiredPRBs[adjacentLeftUser]) {
							selectedUser = adjacentLeftUser;
							Allocated[i] = true;
							MAllocation[i] = selectedUser;
							users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
									i);
#ifdef Allocation
							std::cout << "allocated RB " << i << " to "
									<< selectedUser << std::endl;
#endif
							availableRBs--;
						}
					}
				}

			}
		}
		i++;
	}
//Affichage

#ifdef Allocation
	for (int i = 0; i < nbOfRBs; i++) {
		std::cout << "Mallocation[" << i << "] =" << MAllocation[i]
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
