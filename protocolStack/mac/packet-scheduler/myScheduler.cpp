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

//#define test
//#define SCHEDULER_DEBUG
//#define Allocation
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
	//std::cout << "delay " << metric << std::endl;
	return metric;
}
double myScheduler::ComputeSchedulingMetric(UserToSchedule* user,
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
	int selectedUser;     // user to be selected for allocation
	int selectedPRB;      // PRB to be selected for allocation
	double bestMetric;    // best metric to identify user/RB combination
	double lowestDelay;	  //lowest delay to identify UE with lowest delay
	int left, right;      // index of left and left PRB's to check
	bool Allocated[nbOfRBs];
	double metrics[nbOfRBs][users->size()];
	int requiredPRBs[users->size()];
	int MatriceOfAllocation[nbOfRBs];
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
	int availableH2HRBs;
	int unallocatedH2HUsers;     // No of H2H users which remain unallocated
	int unallocatedM2MUsers;
	double delay[users->size()];
	std::vector<UserToSchedule*> M2MUsers;
	std::vector<UserToSchedule*> H2HUsers;
	//Some initialization
	nbrOfScheduledUsers = 0;
	availableRBs = nbOfRBs;
	availableH2HRBs = 0.6 * nbOfRBs;

	for (int i = 0; i < nbOfRBs; i++) {
		Allocated[i] = false;
		MatriceOfAllocation[i] = -1;
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

	//Differentiate between M2M and H2H devices and create metrics
	for (int j = 0; j < users->size(); j++) {
		scheduledUser = users->at(j);
		if (users->at(j)->m_userToSchedule->GetIDNetworkNode() >= 100
				&& users->at(j)->m_userToSchedule->GetIDNetworkNode() < 500) {
			H2HUsers.push_back(users->at(j));
			RrcEntity *rrc =
					scheduledUser->m_userToSchedule->GetProtocolStack()->GetRrcEntity();
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
#ifdef SCHEDULER_DEBUG
			std::cout << "metrics(H2H) of " << j << std::endl;
			for (int jj = 0; jj < nbOfRBs; jj++) {
				//std::cout  << setw(3) << metrics[jj][ii]/1000 << " ";
				printf("%3d  ", (int) (metrics[jj][j]));
			}
			std::cout << std::endl;
#endif
		} else {
			M2MUsers.push_back(users->at(j));
		}

	}
	unallocatedH2HUsers = H2HUsers.size();
	unallocatedM2MUsers = M2MUsers.size();
	//RBs allocation
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
			MatriceOfAllocation[selectedPRB] =
					H2HUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();

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

						if ((j != selectedUser)
								&& H2HUsers.at(j)->m_listOfAllocatedRBs.size()
										== 0 && requiredPRBs[j] > 0
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
						MatriceOfAllocation[right] =
								H2HUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();
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
						if ((j != selectedUser)
								&& H2HUsers.at(j)->m_listOfAllocatedRBs.size()
										== 0 && requiredPRBs[j] > 0
								&& (metrics[left][selectedUser]
										< metrics[left][j])) {
							ContinueLeft = false;
							break;
						}
					}
					if (ContinueLeft) {
						Allocated[left] = true;
						MatriceOfAllocation[left] =
								H2HUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();
						;
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
			printf("%3d  ", (int) (metrics[jj][j]));
		}
		std::cout << std::endl;

#endif
	}
	while (availableRBs > 0 && unallocatedM2MUsers > 0) //
	{

		//first step: find the best user-RB combinaision
		selectedPRB = -1;
		selectedUser = -1;
		ContinueRight = true;
		ContinueLeft = true;
		lowestDelay = (double) ((1 << 30));
		//Search for the user with lowest delay
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
				for (int j = 0; j < M2MUsers.size(); j++) {
					if ((j != selectedUser)
							&& M2MUsers.at(j)->m_listOfAllocatedRBs.size() == 0
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
							for (int j = 0; j < M2MUsers.size(); j++) {
								if ((j != selectedUser)
										&& M2MUsers.at(j)->m_listOfAllocatedRBs.size()
												== 0 && requiredPRBs[j] > 0
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
							for (int j = 0; j < M2MUsers.size(); j++) {
								if ((j != selectedUser)
										&& M2MUsers.at(j)->m_listOfAllocatedRBs.size()
												== 0 && requiredPRBs[j] > 0
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

			scheduledUser = M2MUsers.at(selectedUser);
			scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);
			Allocated[selectedPRB] = true;
			left = selectedPRB - 1;
			right = selectedPRB + 1;
			ContinueRight = ContinueLeft = true;
			availableRBs--;
			MatriceOfAllocation[selectedPRB] =
					M2MUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();
			while ((scheduledUser->m_listOfAllocatedRBs.size()
					< requiredPRBs[selectedUser])
					&& (ContinueRight || ContinueLeft) && availableRBs > 0) {

				if ((right <= (selectedPRB + nbRBRight[selectedI]))
						&& (scheduledUser->m_listOfAllocatedRBs.size()
								< requiredPRBs[selectedUser])) {
					Allocated[right] = true;
					M2MUsers.at(selectedUser)->m_listOfAllocatedRBs.push_back(
							right);
					MatriceOfAllocation[right] =
							M2MUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();
					right++;
					availableRBs--;
				} else
					ContinueRight = false;
				if ((left >= (selectedPRB - nbRBLeft[selectedI]))
						&& (scheduledUser->m_listOfAllocatedRBs.size()
								< requiredPRBs[selectedUser])) {
					Allocated[left] = true;
					MatriceOfAllocation[left] =
							M2MUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();
					users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
							left);
					left--;
					availableRBs--;
				} else
					ContinueLeft = false;
			}
			/*for (int i = 0; i < scheduledUser->m_listOfAllocatedRBs.size();
			 i++)
			 printf("%d ", scheduledUser->m_listOfAllocatedRBs.at(i));

			 printf("\n------------------\n");*/

			unallocatedM2MUsers--;

		}		//end if selectedUser !=-1
		else
			//no more users to allocate
			break;

	}
//end while

//not all RBs are allocated
/*	int i = 0;

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

			if (left == -1 && MatriceOfAllocation[right] == -1) {
#ifdef Allocation
				std::cout << "RB can't be allocated" << std::endl;
#endif
			}
			//case i =0  or the left RB can't be allocated
			else if (left == -1) {
				selectedUser = MatriceOfAllocation[right];

				std::cout << "user "
						<< users->at(selectedUser)->m_userToSchedule->GetIDNetworkNode()
						<< std::endl;
				int j = right - 1;
				while (j >= 0) {
					if (users->at(selectedUser)->m_listOfAllocatedRBs.size()
							!= requiredPRBs[selectedUser]) {
						Allocated[j] = true;
						MatriceOfAllocation[j] = selectedUser;
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
				selectedUser = MatriceOfAllocation[left];
				int j = i;
				while (j < nbOfRBs) {
					if (users->at(selectedUser)->m_listOfAllocatedRBs.size()
							!= requiredPRBs[selectedUser]) {
						Allocated[j] = true;
						MatriceOfAllocation[j] = selectedUser;
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
				adjacentRightUser = MatriceOfAllocation[right];
				adjacentLeftUser = MatriceOfAllocation[left];
			M[left]>M[right]
			 * if adjacentLeft didn't reach its maximum so RB is allocated to adjacentLeft
			* * else
			* *** if adjacent Right didn't reach its maximum so RB is allocated to adjacent Right
			* *** else RB is not allocated


			if (MatriceOfAllocation[left] != -1) {
				if (metrics[i][adjacentLeftUser]
						>= metrics[i][adjacentRightUser]) {
					if (users->at(adjacentLeftUser)->m_listOfAllocatedRBs.size()
							< requiredPRBs[adjacentLeftUser]) {
						selectedUser = adjacentLeftUser;
						Allocated[i] = true;
						MatriceOfAllocation[i] = selectedUser;
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
#ifdef SCHEDULER_DEBUG
						std::cout << " allocated "
						<< users->at(adjacentRightUser)->m_listOfAllocatedRBs.size()
						+ right - i << " required "
						<< requiredPRBs[adjacentRightUser]
						<< std::endl;
#endif
						selectedUser = adjacentRightUser;
						Allocated[i] = true;
						MatriceOfAllocation[i] = selectedUser;
						users->at(selectedUser)->m_listOfAllocatedRBs.push_back(
								i);
#ifdef Allocation
						std::cout << "allocated RB " << i << " to "
						<< selectedUser << std::endl;
#endif
						availableRBs--;
					}
				}

				M[left]<M[right]
				 * if adjacentRight didn't reach its maximum so RB is allocated to adjacentRight
				* * else
				* *** if adjacent Leftt didn't reach its maximum so RB is allocated to adjacent Left
				* *** else RB is not allocated
				*

				else {
					if ((users->at(adjacentRightUser)->m_listOfAllocatedRBs.size()
									+ right - i)
							<= requiredPRBs[adjacentRightUser]) {
						selectedUser = adjacentRightUser;
						Allocated[i] = true;
						MatriceOfAllocation[i] = selectedUser;
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
						MatriceOfAllocation[i] = selectedUser;
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
}*/
//Affichage
#ifdef Allocation
for (int i = 0; i < nbOfRBs; i++) {
	std::cout << "MatriceOfAllocation[" << i << "] =" << MatriceOfAllocation[i]
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
				scheduledUser1->m_listOfAllocatedRBs.size(), scheduledUser1);
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
#ifdef SCHEDULER_DEBUG
std::cout << "number of scheduled users per TTI " << nbrOfScheduledUsers
		<< std::endl;
#endif
} //end RB Allocation
