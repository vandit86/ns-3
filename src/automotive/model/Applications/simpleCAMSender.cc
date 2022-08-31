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
#include "simpleCAMSender.h"

#include "ns3/CAM.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"


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
            .AddAttribute ("RealTime", "To compute properly timestamps", BooleanValue (false),
                           MakeBooleanAccessor (&simpleCAMSender::m_real_time),
                           MakeBooleanChecker ())
            .AddAttribute ("Client", "TraCI client for SUMO", PointerValue (0),
                           MakePointerAccessor (&simpleCAMSender::m_client),
                           MakePointerChecker<TraciClient> ())
            .AddAttribute ("IsRSU", "Is this node stationary RSU", BooleanValue (false),
                           MakeBooleanAccessor (&simpleCAMSender::m_is_rsu), MakeBooleanChecker ())
            .AddAttribute ("RSU_CAM_Interval", "CAM gereration interval by RSU in ms",
                           IntegerValue (100),
                           MakeIntegerAccessor (&simpleCAMSender::m_rsu_cam_interval_ms),
                           MakeIntegerChecker<uint32_t> ())
            .AddAttribute ("Interface", "number of interface to use", IntegerValue (0),
                           MakeIntegerAccessor (&simpleCAMSender::m_iface),
                           MakeIntegerChecker<uint32_t> ())
            // .AddAttribute ("CAMCallback",
            //                "Callback invoked whenever an CAM is received on this CA application.",
            //                CallbackValue (), 
            //                MakeCallbackAccessor (&simpleCAMSender::m_cam_callback),
            //                MakeCallbackChecker ())
            
            ;

                           
                           
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

      m_id = "rsu-" + std::to_string (this->GetNode()->GetId()); 

    }
    else {
      m_id = m_client->GetVehicleId (this->GetNode ()); // get veh id string 
    }
     
    /** 
     * Create the socket for TX and RX  to send CAMs 
    */
    TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
    m_socket = Socket::CreateSocket (GetNode (), tid);

    // Create new BTP and GeoNet objects and set them in CABasicService
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();
    m_btp->setGeoNet(m_geoNet);
    m_caService.setBTP(m_btp);

    /* Bind the socket to local address */
    PacketSocketAddress local;
    local.SetSingleDevice (GetNode ()->GetDevice (m_iface)->GetIfIndex ());
    local.SetPhysicalAddress (GetNode ()->GetDevice (m_iface)->GetAddress ()); 
    local.SetProtocol (0x8947); // Set GeoNetworking Protocol


    if (m_socket->Bind (local) == -1)
    {
      NS_FATAL_ERROR ("Failed to bind client socket on CAM service");
    }

    /* Set the socket to broadcast */
    PacketSocketAddress remote;
    remote.SetSingleDevice (GetNode ()->GetDevice (m_iface)->GetIfIndex ());
    remote.SetPhysicalAddress (GetNode ()->GetDevice (m_iface)->GetBroadcast ());
    remote.SetProtocol (0x8947);

    m_socket->Connect (remote);

    if (m_is_rsu){
        /* Set callback and station properties in CABasicService 
        // (which will only be used to receive CAMs) ??  */
        m_caService.setStationProperties (999000+(this->GetNode()->GetId()), 
                                                  StationType_roadSideUnit);
        
        // Set the RSU position in the CA and DEN basic service (mandatory for any RSU object)
        // As the position must be specified in (lat, lon), we must take it from the 
        // mobility model and then convert it to Latitude and Longitude
        // As SUMO is used here, we can rely on the TraCIAPI for this conversion
        Ptr<MobilityModel> mob = GetNode ()->GetObject<MobilityModel> ();
        libsumo::TraCIPosition rsuPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (
            mob->GetPosition ().x, mob->GetPosition ().y);
        m_caService.setFixedPositionRSU (rsuPos.y, rsuPos.x);
        
        // interval of cam generation (ms)
        m_caService.changeRSUGenInterval(m_rsu_cam_interval_ms); 
    }
    else {
        m_caService.setStationProperties (1000 + std::stol (m_id.substr (3)),
                                          StationType_passengerCar);
        
        // Vehicle Data Provider nedded for CAM
        VDP *traci_vdp = new VDPTraCI (m_client, m_id);
        m_caService.setVDP (traci_vdp);
    }
    
    /* Set sockets, callback, station properties in CABasicService */
    m_caService.setSocketTx (m_socket);
    m_caService.setSocketRx (m_socket);
    m_caService.addCARxCallback (std::bind(&simpleCAMSender::receiveCAM,this,
                                std::placeholders::_1,std::placeholders::_2));
    m_caService.setRealTime (m_real_time);

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
    (void) cam;
    (void) from; 
    // alterbnative for rssi measurments : run callback when cam from RSU received 

    //  4789:0:1:102:600::200
    //  Ipv6Address r_addr = Inet6SocketAddress::ConvertFrom(from).GetIpv6 (); 
    //  std::cout << "receiveCam, ipv6 : " << r_addr << std::endl;  
    // StationType_t s_type = cam->cam.camParameters.basicContainer.stationType; 
    // //StationID_t s_id = cam->header.stationID;

    // // if our vehicle receive cam from some RSU : invoce callback 
    // if (m_id == "veh1" && s_type == StationType_roadSideUnit) {

    //     long s_id = asn1cpp::getField(cam->header.stationID, long); 
        
    //     double lat = asn1cpp::getField 
    //         (cam->cam.camParameters.basicContainer.referencePosition.latitude, double); 
        
    //     double lon = asn1cpp::getField 
    //         (cam->cam.camParameters.basicContainer.referencePosition.longitude, double); 
        
    //     m_cam_callback (s_id, lat, lon); 
        //cam->cam.camParameters.basicContainer

    // // print only cam my veh receive
    // if (m_id == "veh1" && cam->header.stationID != 1001)
    //   {
    //     std::cout << "get addr " << GetNode ()->GetDevice (0)->GetAddress () << std::endl; 
    //     /* Implement CAM strategy here */
    //     std::cout << "VehicleID: " << m_id << " | Rx CAM from " << cam->header.stationID
    //               // << " | Remote vehicle position: ("
    //               // << asn1cpp::getField (
    //               //        cam->cam.camParameters.basicContainer.referencePosition.latitude, double) /
    //               //        DOT_ONE_MICRO
    //               // << ","
    //               // << asn1cpp::getField (
    //               //        cam->cam.camParameters.basicContainer.referencePosition.longitude,
    //               //        double) /
    //               //        DOT_ONE_MICRO
    //               // << ")" 
                  
    //               << std::endl;
    //   }
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



