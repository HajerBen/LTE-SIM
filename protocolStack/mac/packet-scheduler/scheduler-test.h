/*
 * myScheduler.h
 *
 *  Created on: 26 sept. 2018
 *      Author: baati
 */

#ifndef PSCHEDULERTEST_H_
#define SCHEDULERTEST_H_



#include "uplink-packet-scheduler.h"

class SchedulerTest : public UplinkPacketScheduler {
public:
	SchedulerTest ();
	virtual ~SchedulerTest();

	virtual double ComputeSchedulingMetric (RadioBearer *bearer, double spectralEfficiency, int subChannel);
    virtual double ComputeSchedulingMetric (UserToSchedule* user, int subchannel);

	virtual void RBsAllocation ();
	void SelectFlowsToSchedule ();
	void ContinueAllocation();

};





#endif /* PROTOCOLSTACK_MAC_PACKET_SCHEDULER_MYSCHEDULER_H_ */
