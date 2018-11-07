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
 * Author: Khaled Elsayed <khaled@ieee.org>
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

static void TestMyScheduler ()
{

  srand (time(NULL));

  double flow_duration = 20;
  double maxDelay = 0.1;

  // CREATE COMPONENT MANAGERS
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager* networkManager = NetworkManager::Init();
  //FlowsManager* flowsManager = FlowsManager::Init ();


  //Create cell
  	Cell *cell = new Cell(0, 1, 0.35, 0, 0);
  networkManager->GetCellContainer()->push_back(cell);
  // CREATE CHANNELS and propagation loss model
    LteChannel *dlCh = new LteChannel ();
    LteChannel *ulCh = new LteChannel ();


    // CREATE SPECTRUM
    BandwidthManager* spectrum = new BandwidthManager (3, 3, 0, 0);

  	//Create GW
  	Gateway *gw = new Gateway();
  	networkManager->GetGatewayContainer()->push_back(gw);

  //Create ENodeB
  	ENodeB* enb = new ENodeB (1, cell, 0, 0);
  enb->GetPhy ()->SetDlChannel (dlCh);
  enb->GetPhy ()->SetUlChannel (ulCh);
  enb->GetPhy ()->SetBandwidthManager (spectrum->Copy ());
  enb->SetULScheduler (ENodeB::ULScheduler_TYPE_MYSCHEDULER);
  enb->SetDLScheduler (ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  networkManager->GetENodeBContainer ()->push_back (enb);
  ulCh->AddDevice (enb);


  int nbUEs = 2;

  int idUE = 100;

  int applicationID = 0;
  int srcPort = 0;
  int dstPort = 100;
  int startTime = 0.01; //s
  int stopTime = 10;  //s

  for (int i = 0; i < nbUEs; i++)
    {
	  //ue's random position
	  double posX = (double) (rand() % 1000); //200;
	  		double posY = (double) (rand() % 1000); //200;
	  		double speedDirection = (double) (rand() % 360) * ((2 * 3.14) / 360);
	  		double speed = 30;

	 	  UserEquipment* ue = new UserEquipment (idUE,
	 			                                 posX, posY, 3, speedDirection,
	 			                                 cell,
	 			                                 enb,
	 			                                 0, //handover false!
												 Mobility::CONSTANT_POSITION);

	 	  std::cout << "Created UE - id " << idUE << " position " << posX << " " << posY << std::endl;

	       ue->GetPhy ()->SetDlChannel (dlCh);
	       ue->GetPhy ()->SetUlChannel (ulCh);

	       FullbandCqiManager *cqiManager = new FullbandCqiManager ();
	       cqiManager->SetCqiReportingMode (CqiManager::PERIODIC);
	       cqiManager->SetReportingInterval (1);
	       cqiManager->SetDevice (ue);
	       ue->SetCqiManager (cqiManager);

	       WidebandCqiEesmErrorModel *errorModel = new WidebandCqiEesmErrorModel ();
	       ue->GetPhy ()->SetErrorModel (errorModel);

	       networkManager->GetUserEquipmentContainer ()->push_back (ue);

	       // register ue to the enb
	       enb->RegisterUserEquipment (ue);
	       // define the channel realization
	       MacroCellUrbanAreaChannelRealization* c_dl = new MacroCellUrbanAreaChannelRealization (enb, ue);
	       enb->GetPhy ()->GetDlChannel ()->GetPropagationLossModel ()->AddChannelRealization (c_dl);
	       MacroCellUrbanAreaChannelRealization* c_ul = new MacroCellUrbanAreaChannelRealization (ue, enb);
	       enb->GetPhy ()->GetUlChannel ()->GetPropagationLossModel ()->AddChannelRealization (c_ul);

	       // CREATE UPLINK APPLICATION FOR THIS UE
	             double start_time = 0.5 + (double) (rand() %5);
	             double duration_time = start_time + flow_duration;
	             // create qos parameters
	                	  QoSParameters *qosParameters = new QoSParameters ();
	                	  qosParameters->SetMaxDelay (maxDelay);
	  CBR *cbrApp = new CBR;
	  // create application
	  cbrApp->SetApplicationID(applicationID);
	  cbrApp->SetSource(ue);
	  cbrApp->SetDestination(gw);
	  cbrApp->SetSourcePort(srcPort);
	  cbrApp->SetDestinationPort(dstPort);
	  cbrApp->SetStartTime(start_time);
	  cbrApp->SetStopTime(duration_time);
	  cbrApp->SetInterval (0.04);//en ms
	  cbrApp->SetSize (100);//en bytes
	  cbrApp->SetQoSParameters(qosParameters);

	  ClassifierParameters *cp = new ClassifierParameters (
			  ue->GetIDNetworkNode(),enb->GetIDNetworkNode(),srcPort,
			  dstPort,TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
	  cbrApp->SetClassifierParameters (cp);

	  std::cout << "CREATED CBR APPLICATION, ID " << applicationID << std::endl;


	  idUE++;
	  applicationID++;
	  dstPort++;
	  srcPort++;
/*//Create dL app
  CBR *cbrAppDl = new CBR;
  cbrAppDl->SetApplicationID(applicationID);
 	  cbrAppDl->SetSource(gw);
 	  cbrAppDl->SetDestination(gw);
 	  cbrAppDl->SetStartTime(start_time+0.002);
 	  cbrAppDl->SetStopTime(duration_time);
 	  cbrAppDl->SetInterval (0.04);//en ms
 	  cbrAppDl->SetSize (5);//en bytes
 	  cbrAppDl->SetQoSParameters(qosParameters);
 	 ClassifierParameters *cpDl = new ClassifierParameters (gw->GetIDNetworkNode(),
 	    	                                                       ue->GetIDNetworkNode(),
 	    	                                                       0,
 	    	                                                       10,
 	     	                                                       TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
 	  cbrAppDl->SetClassifierParameters (cpDl);

 	  std::cout << "CREATED CBR APPLICATION, ID " << applicationID << std::endl;

 	 applicationID++;*/

    }



  //Simulator::Init ()->SetStop (0.01);
  Simulator::Init ()->SetStop (10);
  Simulator::Init()->Run ();
}

