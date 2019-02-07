/*
 * myScheduler.h
 *
 *  Created on: 26 sept. 2018
 *      Author: baati
 */

#ifndef PMYSCHEDULER_H_
#define MYSCHEDULER_H_



#include "uplink-packet-scheduler.h"

class myScheduler : public UplinkPacketScheduler {
	std::vector<int> m_M2MScheduled;
public:
	myScheduler ();
	virtual ~myScheduler();

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






#endif /* PROTOCOLSTACK_MAC_PACKET_SCHEDULER_MYSCHEDULER_H_ */
