/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010,2011,2012,2013 TELEMATICS LAB, Politecnico di Bari
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
 * Author: hajer Ben Rekhissa
 */

#include "../channel/LteChannel.h"
#include "../phy/enb-lte-phy.h"
#include "../phy/ue-lte-phy.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../networkTopology/Cell.h"
#include "../protocolStack/packet/packet-burst.h"
#include "../protocolStack/packet/Packet.h"
#include "../core/eventScheduler/simulator.h"
#include "../load-parameters.h"
#include "../flows/application/CBR.h"
#include "../device/IPClassifier/ClassifierParameters.h"
#include "../flows/QoS/QoSParameters.h"
#include "../device/Gateway.h"
#include "../flows/radio-bearer.h"
#include "../channel/propagation-model/channel-realization-helper.h"
#include "../channel/propagation-model/propagation-loss-model.h"
#include "../protocolStack/mac/packet-scheduler/mt-uplink-packet-scheduler.h"
#include "../componentManagers/FlowsManager.h"
#include "../flows/application/Event_Driven_Application.h"
#include "../flows/application/Time_Driven_Application.h"
static void TestUplink(double radius,
			int nbH2H,int nbM2M,
			int sched_Type,
			int stopTime,
			int nbVideo,
			int nbCBR,
			int nbVoIP,
			double bandwidth)
//int frame_struct

		{

	srand(time(NULL));
	int startTime = 0.001;

	// CREATE COMPONENT MANAGERS
	Simulator *simulator = Simulator::Init();
	FrameManager *frameManager = FrameManager::Init();
	NetworkManager* networkManager = NetworkManager::Init();
	FlowsManager* flowsManager = FlowsManager::Init();

	//Create cell
	Cell *cell = new Cell(0, radius, 0.35, 0, 0); //Cell(idCell,radius,minDistance,x,y)
	LteChannel *dlCh = new LteChannel();
	LteChannel *ulCh = new LteChannel();
	networkManager->GetCellContainer()->push_back(cell);
	BandwidthManager* spectrum = new BandwidthManager(bandwidth, bandwidth, 0,
			0);

	//Create GW
	Gateway *gw = new Gateway();
	networkManager->GetGatewayContainer()->push_back(gw);

	//SET SCHEDULING ALLOCATION SCHEME
	ENodeB::ULSchedulerType uplink_scheduler_type;
	switch (sched_Type) {
	case 1:
		uplink_scheduler_type = ENodeB::ULScheduler_TYPE_MAXIMUM_THROUGHPUT;
		std::cout << "Scheduler MT " << std::endl;
		break;
	case 2:
		uplink_scheduler_type = ENodeB::ULScheduler_TYPE_FME;
		std::cout << "Scheduler FME " << std::endl;
		break;
	case 3:
		uplink_scheduler_type = ENodeB::ULScheduler_TYPE_ROUNDROBIN;
		std::cout << "Scheduler RR " << std::endl;
		break;
	case 4:
		uplink_scheduler_type =
				ENodeB::ULScheduler_TYPE_Recursive_Maximum_Expansion;
		std::cout << "Scheduler RME " << std::endl;
		break;
	case 5:
		uplink_scheduler_type = ENodeB::ULScheduler_TYPE_Carrier_By_Carrier;
		std::cout << "Scheduler Carrier By Carrier " << std::endl;
		break;
	case 6:
		uplink_scheduler_type = ENodeB::ULScheduler_TYPE_MYSCHEDULER;
		std::cout << "My Scheduler " << std::endl;
		break;
	case 7:
		uplink_scheduler_type = ENodeB::ULSCHEDULER_TEST;
		std::cout << "scheduler test " << std::endl;
		break;
	case 8:
		uplink_scheduler_type = ENodeB::LIOUMPAS_V2_UPLINK_SCHEDULER;
		std::cout << "LioumpasV2 " << std::endl;
		break;
	default:
		uplink_scheduler_type = ENodeB::ULScheduler_TYPE_FME;
		break;

	}
	//Create ENodeB
	ENodeB* enb = new ENodeB(1, cell);
	enb->GetPhy()->SetDlChannel(dlCh);
	enb->GetPhy()->SetUlChannel(ulCh);
	enb->GetPhy()->SetBandwidthManager(spectrum->Copy());
	enb->SetULScheduler(uplink_scheduler_type);
	enb->SetDLScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
	networkManager->GetENodeBContainer()->push_back(enb);
	ulCh->AddDevice(enb);

	// SET FRAME STRUCTURE
	/*  FrameManager::FrameStructure frame_structure;
	 switch (frame_struct)
	 {
	 case 1:
	 frame_structure = FrameManager::FRAME_STRUCTURE_FDD;
	 break;
	 case 2:
	 frame_structure = FrameManager::FRAME_STRUCTURE_TDD;
	 break;
	 default:
	 frame_structure = FrameManager::FRAME_STRUCTURE_FDD;
	 break;
	 }
	 frameManager->SetFrameStructure(frame_structure);*/

	//Define Application Container
	TraceBased VideoApplication;
	CBR CBRApplication;
	VoIP VoIPApplication;

	int idUe = 100;
	int idM2MUe = 500;
	int applicationID = 0;
	int sourcePort = 0;
	int destinationPort = 100;
	int videoBitRate = 128;

//Create H2H devices
	// create 30 H2H
	for (int i = 0; i < nbH2H; i++) {
		//ue's random position
//	  int maxXY = cell->GetRadius () * 1000;
		int distance = 200;
		double posX = (double) (rand() % 1000);
		double posY = (double) (rand() % 1000);
		double speedDirection = (double) (rand() % 360) * ((2 * 3.14) / 360);
		double speed = 3;

		printf("Creating UE %d at (%lf,%lf)\n", idUe, posX, posY);

		UserEquipment* ue = new UserEquipment(idUe, posX, posY, speed,
				speedDirection, cell, enb, 0, //handover false!
				Mobility::RANDOM_DIRECTION);

		ue->GetPhy()->SetDlChannel(dlCh);
		ue->GetPhy()->SetUlChannel(ulCh);
		ue->GetPhy()->GetDlChannel()->AddDevice(ue);

		WidebandCqiEesmErrorModel *errorModel = new WidebandCqiEesmErrorModel();
		ue->GetPhy()->SetErrorModel(errorModel);
		networkManager->GetUserEquipmentContainer()->push_back(ue);
		//register ue to the enb
		enb->RegisterUserEquipment(ue);
		//define the channel realization
		MacroCellUrbanAreaChannelRealization* c_dl =
				new MacroCellUrbanAreaChannelRealization(enb, ue);
		enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(
				c_dl);
		MacroCellUrbanAreaChannelRealization* c_ul =
				new MacroCellUrbanAreaChannelRealization(ue, enb);
		enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(
				c_ul);

		FullbandCqiManager *cqiManager = new FullbandCqiManager();
		cqiManager->SetCqiReportingMode(CqiManager::PERIODIC);
		cqiManager->SetReportingInterval(1);
		cqiManager->SetDevice(ue);
		ue->SetCqiManager(cqiManager);
		// *** cbr application

		double IAT = 256 / 16000;
		double delay = 0.300;
		double size = 256;
		for (int j = 0; j < nbCBR; j++) {
			//Create an Application
			QoSParameters *qos = new QoSParameters();
			qos->SetMaxDelay(delay);
			CBR *cbrApp = new CBR;
			// create application
			cbrApp->SetApplicationID(applicationID);
			cbrApp->SetSource(ue);
			cbrApp->SetDestination(gw);
			cbrApp->SetSourcePort(sourcePort);
			cbrApp->SetDestinationPort(destinationPort);
			cbrApp->SetStartTime(startTime);
			cbrApp->SetStopTime(stopTime);
			cbrApp->SetInterval(IAT);
			cbrApp->SetSize(size);
			cbrApp->SetQoSParameters(qos);

			ClassifierParameters *cp = new ClassifierParameters(
					ue->GetIDNetworkNode(), gw->GetIDNetworkNode(), sourcePort,
					destinationPort,
					TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
			cbrApp->SetClassifierParameters(cp);

			std::cout << "CREATED CBR APPLICATION, ID " << applicationID
					<< " IAT " << IAT << " Delay " << delay << std::endl;
			//update counter
			destinationPort++;
			applicationID++;
		}

		// create application
		// *** video application
		for (int j = 0; j < nbVideo; j++) {

			VideoApplication.SetSource(ue);
			VideoApplication.SetDestination(enb);
			VideoApplication.SetApplicationID(applicationID);
			VideoApplication.SetStartTime(startTime);
			VideoApplication.SetStopTime(20);

			string video_trace("foreman_H264_");
			//string video_trace ("highway_H264_");
			//string video_trace ("mobile_H264_");

			switch (videoBitRate) {
			case 128: {
				string _file(
						path + "src/flows/application/Trace/" + video_trace
								+ "128k.dat");
				VideoApplication.SetTraceFile(_file);
				std::cout << "		selected video @ 128k" << std::endl;
				break;
			}
			case 242: {
				string _file(
						path + "src/flows/application/Trace/" + video_trace
								+ "242k.dat");
				VideoApplication.SetTraceFile(_file);
				std::cout << "		selected video @ 242k" << std::endl;
				break;
			}
			case 440: {
				string _file(
						path + "src/flows/application/Trace/" + video_trace
								+ "440k.dat");
				VideoApplication.SetTraceFile(_file);
				std::cout << "		selected video @ 440k" << std::endl;
				break;
			}
			default: {
				string _file(
						path + "src/flows/application/Trace/" + video_trace
								+ "128k.dat");
				VideoApplication.SetTraceFile(_file);
				std::cout << "		selected video @ 128k as default" << std::endl;
				break;
			}
			}
			//Create QoS
			QoSParameters *qosVideo = new QoSParameters;
			qosVideo->SetMaxDelay(0.150);
			VideoApplication.SetQoSParameters(qosVideo);
			//create classifier parameters
			ClassifierParameters *cp = new ClassifierParameters(
					ue->GetIDNetworkNode(), gw->GetIDNetworkNode(), sourcePort,
					destinationPort,
					TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
			VideoApplication.SetClassifierParameters(cp);

			std::cout << "CREATED Video APPLICATION, ID " << applicationID
					<< std::endl;
			//update counter
			destinationPort++;
			applicationID++;
			sourcePort++;

		}

		// *** voip application
		for (int j = 0; j < nbVoIP; j++) {
			// create application
			VoIPApplication.SetSource(ue);
			VoIPApplication.SetDestination(gw);
			VoIPApplication.SetApplicationID(applicationID);
			VoIPApplication.SetStartTime(startTime);
			VoIPApplication.SetStopTime(stopTime);

			// create qos parameters

			QoSParameters *qos = new QoSParameters();
			qos->SetMaxDelay(0.1);
			VoIPApplication.SetQoSParameters(qos);

			//create classifier parameters
			ClassifierParameters *cp = new ClassifierParameters(
					ue->GetIDNetworkNode(), gw->GetIDNetworkNode(), 0,
					destinationPort,
					TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
			VoIPApplication.SetClassifierParameters(cp);

			std::cout << "CREATED VOIP APPLICATION, ID " << applicationID
					<< std::endl;

			//update counter
			destinationPort++;
			applicationID++;
		}
		idUe++;
	}

	//Create M2M devices

	for (int i = 0; i < nbM2M; i++) {
		//ue's random position
		//	  int maxXY = cell->GetRadius () * 1000;
		int distance = 200;
		double posX = (double) (rand() % 1000);
		double posY = (double) (rand() % 1000);
		double speedDirection = (double) (rand() % 360) * ((2 * 3.14) / 360);
		double speed = 3;

		printf("Creating UE %d at (%lf,%lf)\n", idM2MUe, posX, posY);

		UserEquipment* ue = new UserEquipment(idM2MUe, posX, posY, speed,
				speedDirection, cell, enb, 0, //handover false!
				Mobility::CONSTANT_POSITION);

		ue->GetPhy()->SetDlChannel(dlCh);
		ue->GetPhy()->SetUlChannel(ulCh);
		ue->GetPhy()->GetDlChannel()->AddDevice(ue);

		WidebandCqiEesmErrorModel *errorModel = new WidebandCqiEesmErrorModel();
		ue->GetPhy()->SetErrorModel(errorModel);
		networkManager->GetUserEquipmentContainer()->push_back(ue);
		//register ue to the enb
		enb->RegisterUserEquipment(ue);
		//define the channel realization
		MacroCellUrbanAreaChannelRealization* c_dl =
				new MacroCellUrbanAreaChannelRealization(enb, ue);
		enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(
				c_dl);
		MacroCellUrbanAreaChannelRealization* c_ul =
				new MacroCellUrbanAreaChannelRealization(ue, enb);
		enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(
				c_ul);

		FullbandCqiManager *cqiManager = new FullbandCqiManager();
		cqiManager->SetCqiReportingMode(CqiManager::PERIODIC);
		cqiManager->SetReportingInterval(1);
		cqiManager->SetDevice(ue);
		ue->SetCqiManager(cqiManager);

		//create time driven devices :70% of M2M application
		if (i < 0.7 * nbM2M) {
			double IAT = (double) (rand() % 5 + 0.05);
			double delay = IAT;
			double size = 125;

			//Create an Application
			QoSParameters *qos = new QoSParameters();
			qos->SetMaxDelay(delay);
			TimeDrivenApplication *TimeDrivenApp = new TimeDrivenApplication;
			// create application
			TimeDrivenApp->SetApplicationID(applicationID);
			TimeDrivenApp->SetSource(ue);
			TimeDrivenApp->SetDestination(gw);
			TimeDrivenApp->SetSourcePort(sourcePort);
			TimeDrivenApp->SetDestinationPort(destinationPort);
			TimeDrivenApp->SetStartTime(startTime);
			TimeDrivenApp->SetStopTime(stopTime);
			TimeDrivenApp->SetInterval(IAT);
			TimeDrivenApp->SetSize(size);
			TimeDrivenApp->SetQoSParameters(qos);

			ClassifierParameters *cp = new ClassifierParameters(
					ue->GetIDNetworkNode(), gw->GetIDNetworkNode(), sourcePort,
					destinationPort,
					TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
			TimeDrivenApp->SetClassifierParameters(cp);

			std::cout << "CREATED Time Driven APPLICATION, ID " << applicationID
					<< " IAT " << IAT << " Delay " << delay << std::endl;
			//update counter
			destinationPort++;
			applicationID++;
		}
		//30% of M2M application are EVENT Driven application
		if (i >= 0.7 * nbM2M) {
			double IAT = (double) (rand() % 5 + 0.05);
			double delay = IAT;
			double size = 125;

			//Create an Application
			QoSParameters *qos = new QoSParameters();
			qos->SetMaxDelay(delay);
			TimeDrivenApplication *EventDrivenApp = new TimeDrivenApplication;
			// create application
			EventDrivenApp->SetApplicationID(applicationID);
			EventDrivenApp->SetSource(ue);
			EventDrivenApp->SetDestination(gw);
			EventDrivenApp->SetSourcePort(sourcePort);
			EventDrivenApp->SetDestinationPort(destinationPort);
			EventDrivenApp->SetStartTime(startTime);
			EventDrivenApp->SetStopTime(stopTime);
			EventDrivenApp->SetInterval(IAT);
			EventDrivenApp->SetSize(size);
			EventDrivenApp->SetQoSParameters(qos);

			ClassifierParameters *cp = new ClassifierParameters(
					ue->GetIDNetworkNode(), gw->GetIDNetworkNode(), sourcePort,
					destinationPort,
					TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
			EventDrivenApp->SetClassifierParameters(cp);

			std::cout << "CREATED Event Driven APPLICATION, ID "
					<< applicationID << " IAT " << IAT << " Delay " << delay
					<< std::endl;
			//update counter
			destinationPort++;
			applicationID++;
		}
		idM2MUe++;
	}
//create Event driven devices :30% of M2M application

	Simulator::Init()->SetStop(stopTime);
	Simulator::Init()->Run();
}

