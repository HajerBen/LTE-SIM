/*
 * FDPS-carrier-by-carrier.cpp
 *
 *  Created on: 18 oct. 2018
 *      Author: hajer
 */

#include "FDPS-carrier-by-carrier.h"
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

CarrierByCarrierPacketScheduler::CarrierByCarrierPacketScheduler() {
	SetMacEntity(0);
	CreateUsersToSchedule();
}

CarrierByCarrierPacketScheduler::~CarrierByCarrierPacketScheduler() {
	Destroy();
}

double CarrierByCarrierPacketScheduler::ComputeSchedulingMetric(
		RadioBearer *bearer, double spectralEfficiency, int subChannel) {
	double metric;
	return metric;
}

double CarrierByCarrierPacketScheduler::ComputeSchedulingMetric(
		UserToSchedule* user, int subchannel) {
	double metric;

	int channelCondition = user->m_channelContition.at(subchannel);
	double spectralEfficiency = GetMacEntity()->GetAmcModule()->GetSinrFromCQI(
			channelCondition);

	metric = spectralEfficiency * 180000;

	return metric;
}

void CarrierByCarrierPacketScheduler::RBsAllocation() {

#ifdef SCHEDULER_DEBUG
	std::cout << " ---- UL RBs Allocation(carrier by carrier)";
#endif
	/* this algorithm implement FDPS carrier by carrier scheduler proposed in "“Proportional Fair Frequency Domain Packet
	 Scheduling For 3GPP LTE Uplink"
	 The FDPS-carrier-by-carrier is a sequential RB allocating algorithm. It allocates
	 the RB’s starting by taking the first RB and finding which user gives the maximum UE-RB value. Once a UE
	 has been assigned a RB, it can no longer be assigned RBs unless it satisfies the contiguity constraint

	 * */
	UsersToSchedule *users = GetUsersToSchedule();
	UserToSchedule* scheduledUser;
	int nbOfRBs =
			GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();

	int availableRBs;     // No of RB's not allocated
	int selectedUe;     // user to be selected for allocation
	double bestMetric;    // best metric to identify user/RB combination
	bool Allocated[nbOfRBs];
	double metrics[nbOfRBs][users->size()];
	int MAllocation[nbOfRBs];
	bool allocatedUser[users->size()];
	int requiredPRBs[users->size()];
	int lastScheduledUe;
	//Some initialization

	//allocatedUser = false;
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

	//Start RB Allocation
	for (int i = 0; i < nbOfRBs; i++) {
		selectedUe = -1;

		bestMetric = (double) (-(1 << 30));
		for (int j = 0; j < users->size(); j++) {
			if (!allocatedUser[j] && requiredPRBs[j] > 0) //only unallocated users requesting some RB's
				if (bestMetric < metrics[i][j]) {
					selectedUe = j;
					bestMetric = metrics[i][j];
				}
		}
		//initialize lastScheduledUe for the first RB
		if (i == 0)
			lastScheduledUe = selectedUe;
		//Compare lastScheduledUE and the new one
		if (lastScheduledUe != selectedUe) {
			allocatedUser[lastScheduledUe] = true;
			lastScheduledUe = selectedUe;
		}
		scheduledUser = users->at(selectedUe);
		scheduledUser->m_listOfAllocatedRBs.push_back(i);
		MAllocation[i] = selectedUe;
	}
	//Affichage
	for (int i = 0; i < nbOfRBs; i++) {
		std::cout << "Mallocation[" << i << "] =" << MAllocation[i]
				<< std::endl;
	}
} //end RB Allocation
