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
#include <algorithm>
//#define SCHEDULER_DEBUG
//#define Allocation
//#define M2Mmetrics

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
double SchedulerTest::ComputeSchedulingMetricCQI(UserToSchedule* user,
		int subChannel) {

	double metric;

	int channelCondition = user->m_channelContition.at(subChannel);

	double spectralEfficiency = GetMacEntity()->GetAmcModule()->GetSinrFromCQI(
			channelCondition);
	metric = spectralEfficiency * 180000;

	return metric;
}

double SchedulerTest::ComputeSchedulingMetricPF(UserToSchedule* user, int RB,
		int NRB) {
	/*
	 * For the PF scheduler the metric is computed
	 * as follows:
	 *
	 * metric = 1- RBi/N_RB
	 * RBi : number of RBs allocated to user i
	 * N_RB is the sum of total number of resources available for allocation at each TTI
	 */

	double metric;

	metric = (double) (1 - (double) RB / NRB);
	return metric;
}

double SchedulerTest::ComputeSchedulingMetricDelay(UserToSchedule* user) {
	double metric; //rest delay

	double maxDelay;
	RrcEntity *rrc = user->m_userToSchedule->GetProtocolStack()->GetRrcEntity();
	for (RrcEntity::RadioBearersContainer::iterator it =
			rrc->GetRadioBearerContainer()->begin();
			it != rrc->GetRadioBearerContainer()->end(); it++) {
		RadioBearer *b = (*it);
		Application* app = b->GetApplication();
		maxDelay = app->GetQoSParameters()->GetMaxDelay();
	}
	metric = maxDelay - user->m_delay;
	return metric;
}

//Sort CQI for each user
void SchedulerTest::ChannelSorted(int SizeOfChannelSorted,
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

bool SchedulerTest::VerifyPower(int nbRB, int requiredPRBs,
		UserToSchedule* scheduledUser) {
	double power;
	bool VerifyPower;
	double Pmax = 23;
	if (nbRB >= requiredPRBs) {
		power = CalculatePower(requiredPRBs, scheduledUser);
	} else {
		power = CalculatePower(nbRB, scheduledUser);
	}
	if (power < Pmax) {
		VerifyPower = true;
	} else
		VerifyPower = false;
	return VerifyPower;
}
void SchedulerTest::RBsAllocation() {
	/* This is an implementation of an algorithm based on  Recursive Maximum Expansion algorithm (RME) reported in
		 * L. Temiño, G. Berardinelli, S. Frattasi,  and P.E. Mogensen,
		 * "Channel-aware scheduling algorithms for SC-FDMA in LTE uplink",  in Proc. PIMRC 2008
		 *
		 */
	#ifdef SCHEDULER_DEBUG
		std::cout << " ----New TTI";
	#endif

		UsersToSchedule *users = GetUsersToSchedule();
		UserToSchedule* scheduledUser;
		int nbOfRBs =
				GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();

		double MPF[nbOfRBs];
		int availableRBs;     // No of RB's not allocated
		int selectedUser;     // user to be selected for allocation
		int selectedPRB;      // PRB to be selected for allocation
		double bestMetric;    // best metric to identify user/RB combination
		double lowestDelay;	  //lowest delay to identify UE with lowest delay
		int left, right;      // index of left and left PRB's to check
		bool Allocated[nbOfRBs];

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
		double power[nbOfRBs];
		double Pmax = 23;
		double delayThreshold = 0.01;

		//****Some initialization****/
		nbrOfScheduledUsers = 0;
		availableRBs = nbOfRBs;
		availableH2HRBs = 0.6 * nbOfRBs;

		for (int i = 0; i < nbOfRBs; i++) {
			Allocated[i] = false;
			MatriceOfAllocation[i] = -1;
		}

		//******Differentiate between M2M and H2H devices
		for (int j = 0; j < users->size(); j++) {
			scheduledUser = users->at(j);

			if (users->at(j)->m_userToSchedule->GetIDNetworkNode() >= 100
					&& users->at(j)->m_userToSchedule->GetIDNetworkNode() < 500) {
				H2HUsers.push_back(users->at(j));

			} else {
				M2MUsers.push_back(users->at(j));

			}
		}

		//create Metrics for H2H and M2M
		double metrics[nbOfRBs][H2HUsers.size()];
		double MCQI[nbOfRBs][M2MUsers.size()];
		int requiredPRBsH2H[H2HUsers.size()];
		int requiredPRBsM2M[M2MUsers.size()];
		//create number of required PRB's per scheduled users

		for (int j = 0; j < H2HUsers.size(); j++) {
			scheduledUser = H2HUsers.at(j);
			RrcEntity *rrc =
					scheduledUser->m_userToSchedule->GetProtocolStack()->GetRrcEntity();
			for (RrcEntity::RadioBearersContainer::iterator it =
					rrc->GetRadioBearerContainer()->begin();
					it != rrc->GetRadioBearerContainer()->end(); it++) {
				RadioBearer *b = (*it);
				Application* app = b->GetApplication();
				b->UpdateAverageTransmissionRate();
				for (int i = 0; i < nbOfRBs; i++) {
					metrics[i][j] = ComputeSchedulingMetricCQI(scheduledUser, i);
				}
			}
			//Calculate number of required PRBS for H2H devices
	#ifdef SCHEDULER_DEBUG1
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
			requiredPRBsH2H[j] = (floor)(
					scheduledUser->m_dataToTransmit
							/ (GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(mcs,
									1) / 8));

	#ifdef SCHEDULER_DEBUG1
			cout << " EffSINR = " << effectiveSinr << "  MCS = " << mcs
			<< "Required RBs " << requiredPRBsH2H[j] << "\n";
	#endif

		}

		for (int j = 0; j < M2MUsers.size(); j++) {
			scheduledUser = M2MUsers.at(j);
			for (int i = 0; i < nbOfRBs; i++) {
				MCQI[i][j] = ComputeSchedulingMetricCQI(scheduledUser, i);
			}
			//Calculate number of required PRBS for M2M devices
	#ifdef SCHEDULER_DEBUG1
			cout << "\n" << "M2M User " << j; // << "CQI Vector";
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
			requiredPRBsM2M[j] = (floor)(
					scheduledUser->m_dataToTransmit
							/ (GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(mcs,
									1) / 8));

	#ifdef SCHEDULER_DEBUG1
			cout << " EffSINR = " << effectiveSinr << "  MCS = " << mcs
			<< "Required RBs " << requiredPRBsM2M[j] << "\n";
	#endif
		}

		unallocatedH2HUsers = H2HUsers.size();
		unallocatedM2MUsers = M2MUsers.size();
		//RBs allocation
		//************Start allocation of H2H devices*****************/

		while (availableH2HRBs > 0 && unallocatedH2HUsers > 0) {
			// First step: find the best user-RB combinaison
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
								&& requiredPRBsH2H[j] > 0) //only unallocated users requesting some RB's
							if (bestMetric < metrics[i][j]) {
								selectedPRB = i;
								selectedUser = j;
								bestMetric = metrics[i][j];
							}
					}

				}
			}
	#ifdef SCHEDULER_DEBUG
			std::cout << "**bestMetric  = " << bestMetric << " selected prb "
					<< selectedPRB << " selected User " << selectedUser
					<< std::endl;
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
						!= requiredPRBsH2H[selectedUser])
						&& (ContinueRight || ContinueLeft) && availableH2HRBs > 0) {
					//start right allocation
					if (right < nbOfRBs && !Allocated[right]) {
						//Verify if the best metric at right doesn't correspond to UE
						for (int j = 0; j < H2HUsers.size(); j++) {

							if ((j != selectedUser)
									&& H2HUsers.at(j)->m_listOfAllocatedRBs.size()
											== 0 && requiredPRBsH2H[j] > 0
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
											== 0 && requiredPRBsH2H[j] > 0
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

		//RBs allocation
		//*********Start M2M Allocation *****/
		/*
		 * *the UE with the lowest delay is prioritized;
		 * 	**if 2 UEs have the same delay the one with best metric (MCQI) is selected
		 * 	* if lowestDelay< delayThreshold => UE is prioritized
		 * 	=> allocate all required RBs
		 * *find the bloc of x RBs with high throughput in MCQI
		 * **if the bloc x allow to allocate the maximum requiredRBs
		 * ***verify if the power < Pmax
		 * ***else find next best bloc x that allow to allocate maximum required RBs and garantee Power <Pmax
		 * *
		 * *
		 */

		for (int j = 0; j < M2MUsers.size(); j++) {
			delay[j] = ComputeSchedulingMetricDelay(M2MUsers.at(j));
	#ifdef SCHEDULER_DEBUG
			std::cout << " delay = " << delay[j] << std::endl;
	#endif
		}

		while (availableRBs > 0 && unallocatedM2MUsers > 0) {

			//first step: find the best user-RB combinaision
			selectedPRB = -1;
			selectedUser = -1;
			ContinueRight = true;
			ContinueLeft = true;
			bestMetric = (double) (-(1 << 30));
			lowestDelay = (double) ((1 << 30));
			//if there are users with delay lowest than delay threshold
			for (int j = 0; j < M2MUsers.size(); j++) {
				if (M2MUsers.at(j)->m_listOfAllocatedRBs.size() == 0
						&& requiredPRBsM2M[j] > 0) //only unallocated users requesting some RB's
								{
					if (delay[j] <= delayThreshold) {
						selectedUser = j;
						lowestDelay = delay[j];
					}

				}
			}
	#ifdef SCHEDULER_DEBUG
			//Affichage of MCQI
			for (int j = 0; j < M2MUsers.size(); j++) {
				std::cout << "metrics of M2M" << j << std::endl;
				for (int i = 0; i < nbOfRBs; i++) {
					std::cout << MCQI[i][j] / 1000 << " ";
				}
				std::cout << "" << std::endl;
			}
	#endif
			bool ContinueM2MAllocation =true;
			// if there is no user with critical delay select the M2M with best CQI
			if (selectedUser == -1) {
				/******/
				if (m_M2MScheduled.size() == M2MUsers.size()) {
					m_M2MScheduled.clear();
					for (int k = 0; k < M2MUsers.size(); k++) {
					if (M2MUsers.at(k)->m_listOfAllocatedRBs.size() > 0) {
						m_M2MScheduled.push_back(
								M2MUsers.at(k)->m_userToSchedule->GetIDNetworkNode());
					}
				}
					if (m_M2MScheduled.size() == M2MUsers.size())
					{
						ContinueM2MAllocation = false;
					}
				}
	//reintialize m_M2MScheduled in order to allocate only not yet allocated users

				//Affichage
	#ifdef SCHEDULER_DEBUG
				for (int k = 0; k < m_M2MScheduled.size(); k++) {
					std::cout << m_M2MScheduled[k] << std::endl;
				}
				std::cout << "****** M2M Users" << std::endl;
				for (int k = 0; k < M2MUsers.size(); k++) {
					std::cout
							<< M2MUsers.at(k)->m_userToSchedule->GetIDNetworkNode()
							<< std::endl;
				}
	#endif
				//Search for the best metric
				for (int i = 0; i < nbOfRBs; i++) {
					if (!ContinueM2MAllocation)
					{
						std::cout << "finish M2M Allocation"<< selectedUser << std::endl;

						break;
					}
					if (!Allocated[i]) { // check only unallocated PRB's
						for (int j = 0; j < M2MUsers.size(); j++) {
							if (M2MUsers.at(j)->m_listOfAllocatedRBs.size() == 0
									&& requiredPRBsM2M[j] > 0
									&& (std::find(m_M2MScheduled.begin(),
											m_M2MScheduled.end(),
											M2MUsers.at(j)->m_userToSchedule->GetIDNetworkNode())
											== m_M2MScheduled.end())
											&& ContinueM2MAllocation) { //only unallocated users requesting some RB's
								if (bestMetric < MCQI[i][j]) {

									selectedUser = j;
									bestMetric = MCQI[i][j];
								}
							}
						}
					}
				}
			}

	#ifdef SCHEDULER_DEBUG
			std::cout << " selected User "
					<< M2MUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode()
					<< " Required PRBs " << requiredPRBsM2M[selectedUser]
					<< std::endl;
	#endif
			//find the best RBs for selected M2M
			//sort Metrics for selected UE

			//initialization
			if (selectedUser != -1) {
				scheduledUser = M2MUsers.at(selectedUser);
				ChannelsSorted.clear();
				bestRBs.clear();
				//initialization
				for (int i = 0; i < nbOfRBs; i++) {
					nbRBRight[i] = 0;
					nbRBLeft[i] = 0;
					nbRB[i] = 0;
				}
				//sort channels
				for (int i = 0; i < nbOfRBs; i++) {
					if (!Allocated[i]) {
						ChannelsSorted.push_back(MCQI[i][selectedUser]);
						bestRBs.push_back(i);
					}
				}

				ChannelSorted(ChannelsSorted.size(), ChannelsSorted, bestRBs);
				//affichage
	#ifdef SCHEDULER_DEBUG
				std::cout << "ChannelsSorted\n ";
				for (int j = 0; j < ChannelsSorted.size(); j++)
					std::cout << ChannelsSorted[j] << " RB " << bestRBs[j]
							<< std::endl;
	#endif

				//find for each best metric the maximum RBs that can be allocated to selectedUE
				for (int i = 0; i < ChannelsSorted.size(); i++) {
					bool Continue = true;
					//verify if the bestRBs[i] can't be allocated to an other UE
					for (int j = 0; j < M2MUsers.size(); j++) {
						if ((j != selectedUser)
								&& (std::find(m_M2MScheduled.begin(),
										m_M2MScheduled.end(),
										M2MUsers.at(j)->m_userToSchedule->GetIDNetworkNode())
										== m_M2MScheduled.end())
								&& requiredPRBsM2M[j] > 0
								&& (MCQI[bestRBs[i]][selectedUser]
										< MCQI[bestRBs[i]][j])) {
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
											&& (std::find(m_M2MScheduled.begin(),
													m_M2MScheduled.end(),
													M2MUsers.at(j)->m_userToSchedule->GetIDNetworkNode())
													== m_M2MScheduled.end())
											&& requiredPRBsM2M[j] > 0
											&& (MCQI[k][selectedUser] < MCQI[k][j])) {
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
											&& (std::find(m_M2MScheduled.begin(),
													m_M2MScheduled.end(),
													M2MUsers.at(j)->m_userToSchedule->GetIDNetworkNode())
													== m_M2MScheduled.end())
											&& requiredPRBsM2M[j] > 0
											&& (MCQI[k][selectedUser] < MCQI[k][j])) {
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

	#ifdef SCHEDULER_DEBUG
					std::cout << "nb of RBs = " << nbRB[i] << std::endl;

	#endif
				} //end calculation of possible allocatedRB for each CQI
				  //Verify if ue has best RB/CQI
				bool Verify = false;
				for (int i = 0; i < ChannelsSorted.size(); i++) {
					if (nbRB[i] > 0) {
						Verify = true;
						break;
					}
				}
				//if the selected UE has no RB with best CQI
				if (Verify == false) {
					selectedPRB = bestRBs[0];
				}
				//Choisir le CQI qui permet d'avoir le min transmission power

				bool VerifyPF;
				bool verifyPower;

				for (int i = 0; i < ChannelsSorted.size() && Verify; i++) {
					VerifyPF = true;
					verifyPower = false;
					verifyPower = VerifyPower(nbRB[i],
							requiredPRBsM2M[selectedUser], scheduledUser);
					if ((nbRB[i] >= requiredPRBsM2M[selectedUser]) && verifyPower) {
						VerifyPF = true;
					} else {
						for (int j = i + 1; j < ChannelsSorted.size(); j++) {
							if (nbRB[j] > nbRB[i]) {
								verifyPower = VerifyPower(nbRB[j],
										requiredPRBsM2M[selectedUser],
										scheduledUser);
								if (verifyPower) {
									VerifyPF = false;
									break;
								} else
									VerifyPF = true;

							} else
								VerifyPF = true;
						}
					}

					if (VerifyPF) {
						selectedPRB = bestRBs[i];
						selectedI = i;
						break;
					}
				}
	#ifdef SCHEDULER_DEBUG
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
				//*****if the Ue has delay < delay_threshold
				if (lowestDelay <= delayThreshold) {
					while ((scheduledUser->m_listOfAllocatedRBs.size()
							< requiredPRBsM2M[selectedUser])
							&& (ContinueRight || ContinueLeft) && availableRBs > 0) {
						if ((right <= nbOfRBs - 1) && !Allocated[right]
								&& (scheduledUser->m_listOfAllocatedRBs.size()
										< requiredPRBsM2M[selectedUser])) {
							Allocated[right] = true;
							M2MUsers.at(selectedUser)->m_listOfAllocatedRBs.push_back(
									right);
							MatriceOfAllocation[right] =
									M2MUsers.at(selectedUser)->m_userToSchedule->GetIDNetworkNode();
							right++;
							availableRBs--;
						} else
							ContinueRight = false;
						if ((left >= 0) && !Allocated[left]
								&& (scheduledUser->m_listOfAllocatedRBs.size()
										< requiredPRBsM2M[selectedUser])) {
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
				}
				if (Verify) {
					while ((scheduledUser->m_listOfAllocatedRBs.size()
							< requiredPRBsM2M[selectedUser])
							&& (ContinueRight || ContinueLeft) && availableRBs > 0) {
						if ((right <= (selectedPRB + nbRBRight[selectedI]))
								&& (scheduledUser->m_listOfAllocatedRBs.size()
										< requiredPRBsM2M[selectedUser])) {
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
										< requiredPRBsM2M[selectedUser])) {
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
				}

			m_M2MScheduled.push_back(
						scheduledUser->m_userToSchedule->GetIDNetworkNode());
						#ifdef SCHEDULER_DEBUG
				printf("Scheduled User = %d Required RBs = %d Allocated RB's= %d\n",
						scheduledUser->m_userToSchedule->GetIDNetworkNode(),
						requiredPRBsM2M[selectedUser],
						scheduledUser->m_listOfAllocatedRBs.size());
				for (int k = 0; k < scheduledUser->m_listOfAllocatedRBs.size(); k++)
					printf("%d \n", scheduledUser->m_listOfAllocatedRBs.at(k));

				for (int t = 0; t < nbOfRBs; t++) {
					std::cout << MatriceOfAllocation[t] << " ";
				}

				printf("\n------------------\n");
	#endif
			} else
				//"no more users to allocate"
				break;

		}	//end while



		//Calculate power
		for (int j = 0; j < users->size(); j++) {
			UserToSchedule* scheduledUser1;
			scheduledUser1 = users->at(j);
			scheduledUser1->m_transmittedData =
					GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(
							scheduledUser1->m_selectedMCS,
							scheduledUser1->m_listOfAllocatedRBs.size()) / 8;
			if (scheduledUser1->m_listOfAllocatedRBs.size() == 0){
				scheduledUser1->m_power = 0;
				scheduledUser1->m_power2 = 0;
			}

			else
			{
				scheduledUser1->m_power = CalculatePower(
						scheduledUser1->m_listOfAllocatedRBs.size(),
						scheduledUser1);
				scheduledUser1->m_power2 = CalculatePower2(
									scheduledUser1->m_listOfAllocatedRBs.size(),
									scheduledUser1);
			}
	#ifdef SCHEDULER_DEBUG1
			printf("Scheduled User = %d mcs = %d Allocated RB's= %d\n",
					scheduledUser1->m_userToSchedule->GetIDNetworkNode(),
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
	}			//end RB Allocation

