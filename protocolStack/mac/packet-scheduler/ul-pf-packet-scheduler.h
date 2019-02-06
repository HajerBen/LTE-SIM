/*
 * ul-pf-packet-schduler.h
 *
 *  Created on: 19 déc. 2018
 *      Author: hajer
 */

#ifndef PROTOCOLSTACK_MAC_PACKET_SCHEDULER_UL_PF_PACKET_SCHEDULER_H_
#define PROTOCOLSTACK_MAC_PACKET_SCHEDULER_UL_PF_PACKET_SCHEDULER_H_

#include "uplink-packet-scheduler.h"

class UlPFScheduler : public UplinkPacketScheduler {
public:
	UlPFScheduler ();
	virtual ~UlPFScheduler();

	virtual double ComputeSchedulingMetric (RadioBearer *bearer, double spectralEfficiency, int subChannel);
    virtual double ComputeSchedulingMetric (UserToSchedule* user, int subchannel);
   	virtual void RBsAllocation ();

//HB
   	double ComputeSchedulingMetric(UserToSchedule* user,RadioBearer *bearer, int subChannel);

};






#endif /* PROTOCOLSTACK_MAC_PACKET_SCHEDULER_UL_PF_PACKET_SCHEDULER_H_ */
