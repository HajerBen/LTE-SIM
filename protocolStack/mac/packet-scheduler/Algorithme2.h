/*
 * Algorithme2.h
 *
 *  Created on: 20 janv. 2019
 *      Author: hajer
 */

#ifndef PROTOCOLSTACK_MAC_PACKET_SCHEDULER_ALGORITHME2_H_
#define PROTOCOLSTACK_MAC_PACKET_SCHEDULER_ALGORITHME2_H_
#include "uplink-packet-scheduler.h"

class Algorithme2: public UplinkPacketScheduler {
public:
	Algorithme2();
	virtual ~Algorithme2();

	virtual double ComputeSchedulingMetric(RadioBearer *bearer,double spectralEfficiency, int subChannel);
	virtual double ComputeSchedulingMetric(UserToSchedule* user,int subchannel);
	virtual void RBsAllocation();

	//HB
	void ChannelSorted (int SizeOfChannelSorted,std::vector<int> &ChannelsSorted, std::vector<int> &bestRBs);
	double ComputeSchedulingMetricCQI(UserToSchedule* user, int subChannel);
	double ComputeSchedulingMetricDelay(UserToSchedule* user);
	double ComputeSchedulingMetricPF(UserToSchedule* user,int RB, int NRB);
	void UpdateAverageTransmissionRate (void);
	bool VerifyPower(int nbRB, int requiredPRBs, UserToSchedule* scheduledUser);

};




#endif /* PROTOCOLSTACK_MAC_PACKET_SCHEDULER_ALGORITHME2_H_ */
