/*
 * rme_uplink-packet_scheduler.h
 *
 *  Created on: 10 oct. 2018
 *      Author: hajer
 */

#ifndef PROTOCOLSTACK_MAC_PACKET_SCHEDULER_RME_UPLINK_PACKET_SCHEDULER_H_
#define PROTOCOLSTACK_MAC_PACKET_SCHEDULER_RME_UPLINK_PACKET_SCHEDULER_H_



#include "uplink-packet-scheduler.h"

class RecursiveMaximumExpansion : public UplinkPacketScheduler {
public:
	RecursiveMaximumExpansion ();
	virtual ~RecursiveMaximumExpansion();

	virtual double ComputeSchedulingMetric (RadioBearer *bearer, double spectralEfficiency, int subChannel);
    virtual double ComputeSchedulingMetric (UserToSchedule* user, int subchannel);

	virtual void RBsAllocation ();

};

#endif /* PROTOCOLSTACK_MAC_PACKET_SCHEDULER_RME_UPLINK_PACKET_SCHEDULER_H_ */
