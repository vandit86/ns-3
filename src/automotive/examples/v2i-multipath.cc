/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/* *
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
 */

#include "ns3/abort.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/csma-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/assert.h"
#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include "ns3/applications-module.h" //<-- for app
#include "ns3/netanim-module.h" // <-- for netanim

/*  required for sumo copling */
#include "ns3/automotive-module.h"
#include "ns3/traci-module.h"
#include "ns3/wave-module.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/PRRSupervisor.h"
#include <unistd.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("MpFdExample");

// TODO: remove this function
static void
PingRtt (std::string context, Time rtt)
{
  NS_LOG_UNCOND ("Received Response with RTT = " << rtt);
  //Simulator::GetImplementation();
}

/**
   * Set the address of a previously added UE
   * \brief  we need to add address of lxc container connected through wired (CSMA) connection to the UE because in LENA, in 
   * the downlink, the PGW uses the destination IP address to identify a single UE and the eNB to which it is attached to. 
   * If you specify a destination IP address that does not belong to any UE, 
   * the packet will just be dropped by the PGW, because it would not know to which eNB it should be routed 
   * through
   * \param pgw PGW node 
   * \param ueLteNetDev lteNetDevice installed on the UE (nedded to get IMSI)
   * \param ueAddr the IPv4 address of the LXC container connected to ue Net device
*/
void
addBackAddress (Ptr<Node> pgw, Ptr<NetDevice> ueLteNetDev, Ipv4Address addr)
{
  // First we need to get IMSI of ueLteNetDevice conencted through csma link to container
  Ptr<LteUeNetDevice> uedev = ueLteNetDev->GetObject<LteUeNetDevice> ();
  uint64_t imsi = uedev->GetImsi ();

  // get PGW application from pgw node
  Ptr<EpcPgwApplication> pgwApp = pgw->GetApplication (0)->GetObject<EpcPgwApplication> ();

  // add container address to allow traffic through PGW node
  pgwApp->SetUeAddress (imsi, addr);
}

/**
 * \brief Calculate and print realtime execution jitter time 
 * \param rt RealtimeSimulatorImpl pointer   
 */
void
JitterMonitor (Ptr<RealtimeSimulatorImpl> rt)
{

  Time mc = rt->Now ();
  Time ts = rt->RealtimeNow ();
  Time j = (ts >= mc) ? ts - mc : mc - ts;
  std::cout << j.GetMicroSeconds () << std::endl;
  Simulator::Schedule (MilliSeconds (100), &JitterMonitor, rt);
}

/**
 * Read from the mob_trace the number of vehicles that will be created.
  *       The number of vehicles is directly parsed from the rou.xml file, looking at all
  *       the valid XML elements of type <vehicle 
*/
int
getNumberVeh (std::string path)
{
  /* Load the .rou.xml document */
  xmlInitParser ();
  xmlDocPtr rou_xml_file;
  rou_xml_file = xmlParseFile (path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR ("Error: unable to parse the specified XML file: " << path);
    }
  int numberOfNodes = -1;
  numberOfNodes = XML_rou_count_vehicles (rou_xml_file);
  xmlFreeDoc (rou_xml_file);
  xmlCleanupParser ();
  if (numberOfNodes == -1)
    {
      NS_FATAL_ERROR (
          "Fatal error: cannot gather the number of vehicles from the specified XML file: "
          << path << ". Please check if it is a correct SUMO file.");
    }
  return numberOfNodes;
}

// ****************************************************************************************************************
// ****************************************************************************************************************
//                                      MAIN
// ****************************************************************************************************************
// ****************************************************************************************************************
int
main (int argc, char *argv[])
{

  // ****************************************
  // Global configurations
  // ****************************************

  // tap devices name (should be created before)
  std::string tap_l ("tap-left");
  std::string tap_l1 ("tap-left-1");
  std::string tap_r ("tap-right");
  std::string tap_r1 ("tap-right-1");
  uint64_t path2delay = 10; // delay between AP and remote host [mks]
  double simTime = 60; // sim time, 1 min by default

  bool sumo_gui = true;
  double sumo_updates = 0.01;
  std::string csv_name;

  // Disabling this option turns off the whole V2X application
  // (useful for comparing the situation when the application is enabled and the one in which it is disabled)
  bool send_cam = true;
  double m_baseline_prr = 150.0;
  bool m_prr_sup = false;
  bool print_summary = false; // To print summary at the end of simulation

  // bool aggregate_out = false;
  // bool vehicle_vis = false;

  //
  // Allow the user to override any of the defaults at run-time, via command-line arguments
  //
  CommandLine cmd (__FILE__);
  cmd.AddValue ("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  // cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  // cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  // cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("summary", "Print a summary for each vehicle at the end of the simulation", print_summary);
  // cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue (
      "send-cam",
      "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",
      send_cam);
  // cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  // cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("prr-sup", "Use the PRR supervisor or not", m_prr_sup);

  cmd.Parse (argc, argv);

  // REAL TIME
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
  // --ns3::RealtimeSimulatorImpl::SynchronizationMode=HardLimit

  // get pointer to rt simulation impl
  ns3::Ptr<ns3::RealtimeSimulatorImpl> realSim =
      ns3::DynamicCast<ns3::RealtimeSimulatorImpl> (ns3::Simulator::GetImplementation ());

  //
  // ****************************************
  // Nodes creation
  // ****************************************
  // Create two ghost nodes.  The first will represent the virtual machine host
  // on the left side of the network; and the second will represent the VM on
  // the right side.
  //

  NodeContainer nodes; // Create two ghost nodes.
  nodes.Create (2);

  NodeContainer nodeAP; // create one wifi AP node
  nodeAP.Create (1);

  NodeContainer enbNode; // create one eNB node
  enbNode.Create (1);

  int numberOfNodes; // number of vehicles from rou.xml file
  NodeContainer obuNodes; // node container for vehicle OBU
  uint32_t nodeCounter = 0; //

  // ****************************************
  // Helpers used in simulation
  // ****************************************
  InternetStackHelper inet; // internet stack helper
  CsmaHelper csma; // Csma Helper
  Ipv4AddressHelper ipv4h; // Ipv4 Address Helper
  Ipv4StaticRoutingHelper ipv4RoutingHelper; // Ipv4 Static Routing Helper

  // ****************************************
  // Add a default internet stack to the node (ARP, IPv4, ICMP, UDP and TCP).
  // ****************************************
  inet.Install (nodeAP);
  inet.Install (nodes);

  // ****************************************************************************************************************
  //                                            MOBILITY MODEL
  // ****************************************************************************************************************
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0, 0, 0)); // pos of left
  positionAlloc->Add (Vector (0, 0, 20)); // pos of AP  --> center of SUMO map
  positionAlloc->Add (Vector (25, 0, 0)); // pos of right
  positionAlloc->Add (Vector (60, 0, 20)); // pos of eNodeB

  MobilityHelper mobility; // mobility helper
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes.Get (0));
  mobility.Install (nodeAP.Get (0));
  mobility.Install (nodes.Get (1));
  mobility.Install (enbNode.Get (0));

  // ****************************************************************************************************************
  //                                  Configure PATH 1: WAVE (802.11p) and CSMA
  // ****************************************************************************************************************

  NodeContainer nodes_l_ap (nodes.Get (0), nodeAP.Get (0));
  NodeContainer nodes_r_ap (nodes.Get (1), nodeAP.Get (0));

  int txPower = 23;
  float datarate = 6;
  std::string phyMode ("OfdmRate6MbpsBW10MHz");

  /*** 2. Create and setup channel   ***/
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  std::cout << "Setting up the 802.11p channel @ " << datarate << " Mbit/s, 10 MHz, and tx power "
            << (int) txPower << " dBm." << std::endl;

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);

  /*** 3. Create and setup MAC ***/
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
                                      StringValue (phyMode), "ControlMode", StringValue (phyMode));

  // Install devices on nodes from path #1
  NetDeviceContainer dev_l_ap = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes_l_ap);

  // configure CSMA AP <--> REMOTE
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (path2delay)));
  NetDeviceContainer dev_r_ap = csma.Install (nodes_r_ap);

  // Assign adress
  ipv4h.SetBase ("16.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_ap_wifi = ipv4h.Assign (dev_l_ap.Get (1));
  Ipv4InterfaceContainer ifce_l_wifi = ipv4h.Assign (dev_l_ap.Get (0));
  ipv4h.SetBase ("17.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_ap_csma = ipv4h.Assign (dev_r_ap.Get (1));
  Ipv4InterfaceContainer ifce_r_csma = ipv4h.Assign (dev_r_ap.Get (0));

  // ****************************************************************************************************************
  //                                  Configure PATH 2: LTE and CSMA
  // ****************************************************************************************************************

  // Default LTE configuration
  Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::RrFfMacScheduler::HarqEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::LteHelper::PathlossModel",
                      StringValue ("ns3::FriisSpectrumPropagationLossModel"));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth",
                      UintegerValue (100)); // 5MHz,  100 for 20MHz bandwidth
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue (100)); //20MHz bandwidth

  // testing
  // Config::SetDefault ("ns3::CcHelper::DlBandwidth", UintegerValue (100)); // 5MHz,  100 for 20MHz bandwidth
  // Config::SetDefault ("ns3::ComponentCarrier::DlBandwidth", UintegerValue (100)); // 5MHz,  100 for 20MHz bandwidth

  Config::SetDefault ("ns3::LteAmc::AmcModel",
                      EnumValue (LteAmc::PiroEW2010)); // Adaptive Modulation and Coding

  // TODO:: change to LteHelper lteHelper instead of CreateObject and move to up

  // get LTE / EPC helpers
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  // Get the pgw node to install the csma device
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Install LTE Devices to the nodes
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNode);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (
      NodeContainer (nodes.Get (0))); // TODO: test without NodeContainer

  // assign ipv4 adresses to UE Devices
  Ipv4InterfaceContainer ifce_l_lte = epcHelper->AssignUeIpv4Address (ueLteDevs);

  // Attach one UE per eNodeB
  // side effect: the default EPS bearer will be activated
  lteHelper->Attach (ueLteDevs.Get (0), enbLteDevs.Get (0));

  // Link: PGW <---> Remote node through CSMA
  csma.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("100Mb/s")));
  csma.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (1)));
  NodeContainer nodes_r_pgw (nodes.Get (1), pgw);
  NetDeviceContainer dev_r_pgw = csma.Install (nodes_r_pgw);

  // and assign ipv4 adress to ifaces
  ipv4h.SetBase ("12.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_pgw_csma = ipv4h.Assign (NetDeviceContainer (dev_r_pgw.Get (1)));
  Ipv4InterfaceContainer ifce_r_csma1 = ipv4h.Assign (NetDeviceContainer (dev_r_pgw.Get (0)));

  // ****************************************************************************************************************
  //                             Setup TRACI/SUMO and configure other Vehicles
  // ****************************************************************************************************************

  /*** 0.a App Options ***/
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2i_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config = "src/automotive/examples/sumo_files_v2i_map/map.sumo.cfg";

  // 0.b parsing xml to get num of veh
  std::cout << "Reading the .rou file..." << std::endl;
  std::string path = sumo_folder + mob_trace;
  numberOfNodes = getNumberVeh (path);                  // parsing sumo xml to get num vehicles
  std::cout << "The .rou file has been read: " << numberOfNodes
            << " vehicles will be present in the simulation." << std::endl;

  /*** 1. Configure dynamic vehicles  ***/
  obuNodes.Create (numberOfNodes - 1); //   (-1) since our UE node is placed in one veh
  if (obuNodes.GetN () > 0)
    {
      mobility.Install (obuNodes);  // install mobility model
      //inet.Install (obuNodes);      // add inet stack
      NetDeviceContainer obuNetDevices =
          wifi80211p.Install (wifiPhy, wifi80211pMac, obuNodes); // install 802.11p devices
      /* Give packet socket powers to nodes (otherwise, if the app tries to create a PacketSocket, 
      CreateSocket will end up with a segmentation fault */
      PacketSocketHelper packetSocket;
      packetSocket.Install (obuNodes);
    }

  /*** 2. Configure sumo cliente  ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue ("")); // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", (BooleanValue) sumo_gui);
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));

  std::string sumo_additional_options = "--verbose true";
  sumo_additional_options += " --collision.action warn --collision.check-junctions "
                             "--error-log=sumo-errors-or-collisions.xml";

  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));

  /*** 3. Create and setup application to run on nodes, app ***/
  areaSpeedAdvisorClient80211pHelper AreaSpeedAdvisorClient80211pHelper;
  Ipv4Address remoteHostAddr;

  // PRR supervisor auxilary object (false by def)
  Ptr<PRRSupervisor> prrSup = NULL;
  PRRSupervisor prrSupObj(m_baseline_prr);
  if(m_prr_sup)
    {
      prrSup = &prrSupObj;
      prrSup->setTraCIClient(sumoClient);
    }

  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("ServerAddr", Ipv4AddressValue (remoteHostAddr));
  AreaSpeedAdvisorClient80211pHelper.SetAttribute (
      "Client",
      (PointerValue) sumoClient); // pass TraciClient object for accessing sumo in application
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("PrintSummary", BooleanValue (print_summary));
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("RealTime", BooleanValue (true));  // working in real time ONLY
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("CSV", StringValue (csv_name));
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("SendCAM", BooleanValue (send_cam));
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("PRRSupervisor", PointerValue (prrSup));

  /* callback function for node creation */
  std::function<Ptr<Node> ()> setupNewWifiNode = [&] () -> Ptr<Node> {
    Ptr<Node> includedNode;
    if (nodeCounter == 0) // veh1 is our UE connected to Left LXC
      {
        includedNode = nodes.Get (0);
        std::string m_vId = "veh1"; // sumo id of our first vehicle
        libsumo::TraCIColor red;
        red.r = 255;
        red.g = 0;
        red.b = 0;
        red.a = 255;

        sumoClient->TraCIAPI::vehicle.setColor (m_vId, red); // set different color to our vehicle
        //sumoClient->vehicle.setColor("veh1",green);
      }
    else
      {
        includedNode = obuNodes.Get (nodeCounter - 1); // get from the node pool (starting from 0)
      }
    nodeCounter++; //
    std::cout << "Created Vehicle ID : " << nodeCounter << ", Node ID : " << includedNode->GetId ()
              << std::endl;

    /* Install Application */
    ApplicationContainer ClientApp = AreaSpeedAdvisorClient80211pHelper.Install (includedNode);
    ClientApp.Start (Seconds (0.0));
    ClientApp.Stop (ns3::Seconds(simTime) - Simulator::Now () - Seconds (0.1));

    return includedNode;
  };

  /* callback function for node shutdown */
  std::function<void (Ptr<Node>)> shutdownWifiNode = [] (Ptr<Node> exNode) {
      /* Stop all applications */
      Ptr<areaSpeedAdvisorClient80211p> areaSpeedAdvisorClient80211p_ = exNode->GetApplication(0)->GetObject<areaSpeedAdvisorClient80211p>();
      if(areaSpeedAdvisorClient80211p_)
        areaSpeedAdvisorClient80211p_->StopApplicationNow ();

    /* Set position outside communication range */
    Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel> ();
    mob->SetPosition (Vector (-1000.0 + (rand () % 25), 320.0 + (rand () % 25),
                              250.0)); // rand() for visualization purposes

    /* NOTE: further actions could be required for a safe shut down! */
  };

  /* Start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  // ****************************************************************************************************************
  //                  Configure FDNetDevices and connect to TAP file descriptor
  // ****************************************************************************************************************

  EmuFdNetDeviceHelper fdNet;
  NetDeviceContainer devices;
  std::string encapMode ("Dix");
  fdNet.SetAttribute ("EncapsulationMode", StringValue (encapMode));

  fdNet.SetDeviceName ("vethLeft");
  devices = fdNet.Install (nodes.Get (0));
  Ptr<NetDevice> leftFdDev = devices.Get (0);
  leftFdDev->SetAttribute ("Address", Mac48AddressValue (Mac48Address::Allocate ()));

  fdNet.SetDeviceName ("vethRight");
  devices = fdNet.Install (nodes.Get (1));
  Ptr<NetDevice> rightFdDev = devices.Get (0);
  rightFdDev->SetAttribute ("Address", Mac48AddressValue (Mac48Address::Allocate ()));

  fdNet.SetDeviceName ("vethLeft1");
  devices = fdNet.Install (nodes.Get (0));
  Ptr<NetDevice> leftFdDev1 = devices.Get (0);
  leftFdDev1->SetAttribute ("Address", Mac48AddressValue (Mac48Address::Allocate ()));

  fdNet.SetDeviceName ("vethRight1");
  devices = fdNet.Install (nodes.Get (1));
  Ptr<NetDevice> rightFdDev1 = devices.Get (0);
  rightFdDev1->SetAttribute ("Address", Mac48AddressValue (Mac48Address::Allocate ()));

  // we have a FdNetDevices connected to TAP devices through fd,
  // now install it on nodes and set IP addresses
  Ptr<Ipv4> ipv4; // get Ipv4 object from node
  Ipv4InterfaceAddress address; // interface address
  uint32_t interface; // ifce number

  // left node (UE)
  ipv4 = nodes.Get (0)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (leftFdDev1);
  address = Ipv4InterfaceAddress (Ipv4Address ("15.0.0.1"), Ipv4Mask ("255.0.0.0"));
  std::cout << "Adress 1: " << address << std::endl;
  ipv4->AddAddress (interface, address);
  //ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  // left node (UE)
  ipv4 = nodes.Get (0)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (leftFdDev);
  address = Ipv4InterfaceAddress (Ipv4Address ("11.0.0.1"), Ipv4Mask ("255.0.0.0"));
  std::cout << "Adress 1: " << address << std::endl;
  ipv4->AddAddress (interface, address);
  //ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  // right node (Remote)
  ipv4 = nodes.Get (1)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (rightFdDev1);
  address = Ipv4InterfaceAddress (Ipv4Address ("14.0.0.1"), Ipv4Mask ("255.0.0.0"));
  std::cout << "Adress 2: " << address << std::endl;
  ipv4->AddAddress (interface, address);
  //ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  // right node (Remote)
  ipv4 = nodes.Get (1)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (rightFdDev);
  address = Ipv4InterfaceAddress (Ipv4Address ("13.0.0.1"), Ipv4Mask ("255.0.0.0"));
  std::cout << "Adress 2: " << address << std::endl;
  ipv4->AddAddress (interface, address);
  //ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  // ****************************************************************************************************************
  //                        Configure ROUTING
  // ****************************************************************************************************************

  //Adding the network behind the UE and remote host to the AP routing table
  // hardcoding the IP address of the LXC interfaces connected to this path
  // TODO :::: change routing

  // *********************************
  // routing on AP node
  // *********************************
  Ptr<Ipv4StaticRouting> apStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (nodeAP.Get (0)->GetObject<Ipv4> ());
  apStaticRouting->AddNetworkRouteTo (Ipv4Address ("15.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                      Ipv4Address ("16.0.0.2"), ifce_ap_wifi.Get (0).second);
  apStaticRouting->AddNetworkRouteTo (Ipv4Address ("14.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                      Ipv4Address ("17.0.0.2"), ifce_ap_csma.Get (0).second);

  // ********************************
  // routing on left node (UE)
  // ********************************
  Ptr<Ipv4StaticRouting> ueStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (nodes.Get (0)->GetObject<Ipv4> ());
  // Set the default gateway for the UE using a static routing
  ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (),
                                    ifce_l_lte.Get (0).second);

  // add route to right lxc through wifi
  ueStaticRouting->AddNetworkRouteTo (Ipv4Address ("14.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                      Ipv4Address ("16.0.0.1"), ifce_l_wifi.Get (0).second);

  // ********************************
  // routing on right node (Remote)
  // ********************************
  Ptr<Ipv4StaticRouting> rightStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (nodes.Get (1)->GetObject<Ipv4> ());
  // default route through LTE path
  rightStaticRouting->SetDefaultRoute (Ipv4Address ("12.0.0.1"), ifce_r_csma1.Get (0).second);
  // add route to left lxc through wifi
  rightStaticRouting->AddNetworkRouteTo (Ipv4Address ("15.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                         Ipv4Address ("17.0.0.1"), ifce_r_csma.Get (0).second);
  // add route to left lxc through wifi
  // do non need route to 16. just for ping test
  rightStaticRouting->AddNetworkRouteTo (Ipv4Address ("16.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                         Ipv4Address ("17.0.0.1"), ifce_r_csma.Get (0).second);

  // ********************************
  // routing on PGW node
  // ********************************
  // Adding the network behind the UE to the pgw
  // hardcoding the IP address of the UE connected to the external network
  Ptr<Ipv4StaticRouting> pgwStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (pgw->GetObject<Ipv4> ());
  pgwStaticRouting->AddNetworkRouteTo (Ipv4Address ("11.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                       Ipv4Address ("7.0.0.2"), 1);
  pgwStaticRouting->AddNetworkRouteTo (Ipv4Address ("13.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                       Ipv4Address ("12.0.0.2"), ifce_pgw_csma.Get (0).second);

  // Add LXC left address to allow "ping" through EPC-PGW node
  addBackAddress (pgw, ueLteDevs.Get (0), Ipv4Address ("11.0.0.2"));

  // ****************************************************************************************************************
  //                        Configure APPLICATIONS
  // ****************************************************************************************************************
  //
  // Create the ping application.  This application knows how to send
  // ICMP echo requests.  Setting up the packet sink manually is a bit
  // of a hassle and since there is no law that says we cannot mix the
  // helper API with the low level API, let's just use the helper.
  //
  // std::string remote ("14.0.0.2");
  // std::string mask ("255.0.0.0");
  // std::string pi ("no");
  // NS_LOG_INFO ("Create V4Ping Appliation");
  // TODO:: remove or change
  // Ipv4Address remoteIp ("14.0.0.2");
  // Ptr<V4Ping> app = CreateObject<V4Ping> ();
  // app->SetAttribute ("Remote", Ipv4AddressValue (remoteIp));
  // app->SetAttribute ("Verbose", BooleanValue (true));
  // nodes.Get(0)->AddApplication (app);
  // app->SetStartTime (Seconds (1.0));
  // app->SetStopTime (Seconds (simTime-1));

  // //
  // // Give the application a name.  This makes life much easier when constructing
  // // config paths.
  // //
  // Names::Add ("app", app);
  // //
  // // Hook a trace to print something when the response comes back.
  // //
  // Config::Connect ("/Names/app/Rtt", MakeCallback (&PingRtt));

  wifiPhy.EnablePcapAll ("mp-wifi", true);
  csma.EnablePcapAll ("mp-csma", true);
  fdNet.EnablePcapAll ("mp-fd", true);

  // ********************************************************
  // Debug: Testing that proper IP addresses are configured
  // ********************************************************

  // realtime jitter calculation :
  // change interval
  // Simulator::Schedule (MilliSeconds(100), &JitterMonitor, realSim);

  // config output , write config params to file
  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
  ConfigStore outputConfig;
  outputConfig.ConfigureDefaults ();
  outputConfig.ConfigureAttributes ();

  /*** 10. Setup and Start Simulation + Animation ***/
  // AnimationInterface anim ("ns3-sumo-coupling.xml"); // Mandatory
  //
  // Now, do the actual emulation.
  //
  NS_LOG_INFO ("Run Emulation.");
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
}
