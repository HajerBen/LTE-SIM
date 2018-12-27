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
public:
	myScheduler ();
	virtual ~myScheduler();

	virtual double ComputeSchedulingMetric (RadioBearer *bearer, double spectralEfficiency, int subChannel);
    virtual double ComputeSchedulingMetric (UserToSchedule* user, int subchannel);
   	virtual void RBsAllocation ();

//HB
   	double ComputeSchedulingMetric(UserToSchedule* user,RadioBearer *bearer, int subChannel);
	double ComputeSchedulingMetric(UserToSchedule* user) ;
	void ChannelSorted(int SizeOfChannelSorted,std::vector<int> &ChannelsSorted, std::vector<int> &bestRBs );
};





#endif /* PROTOCOLSTACK_MAC_PACKET_SCHEDULER_MYSCHEDULER_H_ */
