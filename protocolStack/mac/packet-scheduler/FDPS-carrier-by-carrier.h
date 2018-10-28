/*
 * FDPS-carrier-by-carrier.h
 *
 *  Created on: 18 oct. 2018
 *      Author: hajer
 */

#ifndef PROTOCOLSTACK_MAC_PACKET_SCHEDULER_FDPS_CARRIER_BY_CARRIER_H_
#define PROTOCOLSTACK_MAC_PACKET_SCHEDULER_FDPS_CARRIER_BY_CARRIER_H_


#include "uplink-packet-scheduler.h"

class CarrierByCarrierPacketScheduler : public UplinkPacketScheduler {
public:
	CarrierByCarrierPacketScheduler();
	virtual ~CarrierByCarrierPacketScheduler();

	virtual double ComputeSchedulingMetric (RadioBearer *bearer, double spectralEfficiency, int subChannel);
    virtual double ComputeSchedulingMetric (UserToSchedule* user, int subchannel);
	virtual void RBsAllocation ();

};

#endif /* CBC_UPLINK_PACKET_SCHEDULER_H_ */
