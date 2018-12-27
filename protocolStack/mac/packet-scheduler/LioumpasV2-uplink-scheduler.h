/*
 * Lioumpas.h
 *
 *  Created on: 16 nov. 2018
 *      Author: hajer
 */

#ifndef PROTOCOLSTACK_MAC_PACKET_SCHEDULER_LIOUMPASV2_UPLINK_SCHEDULER_H_
#define PROTOCOLSTACK_MAC_PACKET_SCHEDULER_LIOUMPASV2_UPLINK_SCHEDULER_H_



#include "uplink-packet-scheduler.h"

class LioumpasV2: public UplinkPacketScheduler {
public:
	LioumpasV2 ();
	virtual ~LioumpasV2();

	virtual double ComputeSchedulingMetric (RadioBearer *bearer, double spectralEfficiency, int subChannel);
    virtual double ComputeSchedulingMetric (UserToSchedule* user, int subchannel);
   	virtual void RBsAllocation ();

//HB
	double ComputeSchedulingMetric(UserToSchedule* user) ;
	double ComputeSchedulingMetric(UserToSchedule* user,RadioBearer *bearer, int subChannel);
};




#endif /* PROTOCOLSTACK_MAC_PACKET_SCHEDULER_LIOUMPASV2_UPLINK_SCHEDULER_H_ */
