/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Giuseppe Avino, Politecnico di Torino (giuseppe.avino@polito.it)
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/
#include "simpleCAMSender-gps-tc.h"

#include "ns3/CAM.h"
#include "ns3/DENM.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"


// #include "ns3/CAM.h"
// #include "ns3/DENM.h"
// #include "ns3/Seq.hpp"
// #include "ns3/Getter.hpp"
// #include "ns3/Setter.hpp"
// #include "ns3/Encoding.hpp"
// #include "ns3/SetOf.hpp"
// #include "ns3/SequenceOf.hpp"
// #include "ns3/socket.h"
// #include "ns3/btpdatarequest.h"
// #include "ns3/network-module.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("simpleCAMSender");

  NS_OBJECT_ENSURE_REGISTERED(simpleCAMSender);

  TypeId
  simpleCAMSender::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::simpleCAMSender")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<simpleCAMSender> ()
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&simpleCAMSender::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("Client",
            "TraCI client for SUMO",
            PointerValue (0),
            MakePointerAccessor (&simpleCAMSender::m_client),
            MakePointerChecker<TraciClient> ())
        .AddAttribute ("IsRSU",
            "Is this node stationary RSU",
            BooleanValue(false),
            MakeBooleanAccessor (&simpleCAMSender::m_is_rsu),
            MakeBooleanChecker ());
        return tid;
  }

  simpleCAMSender::simpleCAMSender ()
  {
    NS_LOG_FUNCTION(this);
    m_client = nullptr;

    m_cam_sent = 0;
  }

  simpleCAMSender::~simpleCAMSender ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  simpleCAMSender::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  simpleCAMSender::StartApplication (void)
  {

    /*
     * This application simply generates CAM.
     * In this case, the position of the vehicle will arrive from FROM SUMO!
     */

    NS_LOG_FUNCTION(this);

    // Ensure that the mobility client has been set
    if(m_client==nullptr)
    {
        NS_FATAL_ERROR("No mobility client specified in simpleCAMSender");
    }

    // if it's stationary RSU don't use TraCI to get ID 
    if (m_is_rsu){

    }
    else {
      m_id = m_client->GetVehicleId (this->GetNode ()); // get veh id string 
    }
     
    /** 
     * Create the socket for TX and RX  to send CAMs and receive DENMs 
    */
    TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
    m_socket = Socket::CreateSocket (GetNode (), tid);

    // Create new BTP and GeoNet objects and set them in CABasicService
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();
    m_btp->setGeoNet(m_geoNet);
    //m_denService.setBTP(m_btp);
    m_caService.setBTP(m_btp);

    /****/

    /* Bind the socket to local address */
    PacketSocketAddress local;
    local.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
    local.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetAddress ()); 
    local.SetProtocol (0x8947); // Set GeoNetworking Protocol

    if (m_socket->Bind (local) == -1)
    {
      NS_FATAL_ERROR ("Failed to bind client socket");
    }

    /* Set the socket to broadcast */
    PacketSocketAddress remote;
    remote.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
    remote.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetBroadcast ());
    remote.SetProtocol (0x8947);

    m_socket->Connect (remote);

    /*************/

    /* Set sockets, callback and station properties in DENBasicService */
    //m_denService.setSocketTx (m_socket);
    //m_denService.setSocketRx (m_socket);

    // m_denService.setStationProperties (std::stol(m_id), StationType_passengerCar);

    // m_denService.addDENRxCallback (std::bind(&simpleCAMSender::receiveDENM,this,std::placeholders::_1,std::placeholders::_2));
    // m_denService.setRealTime (m_real_time);

    if (m_is_rsu){
        /* Set callback and station properties in CABasicService 
        // (which will only be used to receive CAMs) ??  */
        m_caService.setStationProperties (777888999, StationType_roadSideUnit);
        
        // Set the RSU position in the CA and DEN basic service (mandatory for any RSU object)
        // As the position must be specified in (lat, lon), we must take it from the 
        // mobility model and then convert it to Latitude and Longitude
        // As SUMO is used here, we can rely on the TraCIAPI for this conversion
        Ptr<MobilityModel> mob = GetNode ()->GetObject<MobilityModel> ();
        libsumo::TraCIPosition rsuPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (
            mob->GetPosition ().x, mob->GetPosition ().y);
        m_caService.setFixedPositionRSU (rsuPos.y, rsuPos.x);
    }
    else {
        m_caService.setStationProperties (1000 + std::stol (m_id.substr (3)),
                                          StationType_passengerCar);
        // Somthing for GeoNet functionality (TraCI VDP )
        VDP *traci_vdp = new VDPTraCI (m_client, m_id);
        m_caService.setVDP (traci_vdp);
    }
    
    /* Set sockets, callback, station properties in CABasicService */
    m_caService.setSocketTx (m_socket);
    m_caService.setSocketRx (m_socket);
    m_caService.addCARxCallback (std::bind(&simpleCAMSender::receiveCAM,this,
                                std::placeholders::_1,std::placeholders::_2));
    m_caService.setRealTime (m_real_time);

    //VDP* gpstc_vdp = new VDPGPSTraceClient(m_gps_tc_client,m_id);
    // m_caService.setVDP(gpstc_vdp);
    // m_denService.setVDP(gpstc_vdp);


    /* Schedule CAM dissemination */
    std::srand(Simulator::Now().GetNanoSeconds ());
    double desync = ((double)std::rand()/RAND_MAX);
    m_caService.startCamDissemination(desync);
  }

  void
  simpleCAMSender::StopApplication ()
  {
    NS_LOG_FUNCTION(this);

    uint64_t cam_sent;
    cam_sent = m_caService.terminateDissemination ();

    std::cout << "Vehicle " << m_id
              << " has sent " << cam_sent
              << " CAMs" << std::endl;
       
  }

  void
  simpleCAMSender::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  simpleCAMSender::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
  {
    /* Implement CAM strategy here */
    std::cout <<"VehicleID: " << m_id
            <<" | Rx CAM from "<<cam->header.stationID
            <<" | Remote vehicle position: ("<<asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/DOT_ONE_MICRO<<","
            <<asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/DOT_ONE_MICRO<<")"<<std::endl;

  }

  void
  simpleCAMSender::receiveDENM (denData denm, Address from)
  {
    /* This is just a sample dummy receiveDENM function. The user can customize it to parse the content of a DENM when it is received. */
    (void) denm; // Contains the data received from the DENM
    (void) from; // Contains the address from which the DENM has been received
    std::cout<<"Received a new DENM."<<std::endl;
  }

}



