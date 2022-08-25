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
#include "ns3/netanim-module.h" //<-- for netanim

/*  required for sumo copling */
#include "ns3/automotive-module.h"
#include "ns3/traci-module.h"
#include "ns3/wave-module.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/PRRSupervisor.h"

#include <sstream>      // std::stringstream
#include <unistd.h> 

// interact with mptcpd-plugin 
#include "/home/vad/mptcpd/include/mptcpd/mptcp_ns3.h"
#include <fcntl.h>

using namespace ns3;
/* we run in real-time , so no loggin is avilable  */
//NS_LOG_COMPONENT_DEFINE ("MpFdExample");

#define MAX_VEH_SPEED 120 // km/h
#define MIN_VEH_SPEED 10  // Km/h
#define MAX_JITTER    100 // ms

// Global variables for use in callbacks.
double g_signalDbm;
double average_RSU_conn_time; 
double simTime = 60;          // sim time, 1 min by default

//double g_noiseDbmAvg;
//uint32_t g_samples;

/** NS-3 connection glob vars **/
        
int mptcpdFd;      // file descriptor to interact with mptcpd             

/**
   * Set the address of a previously added UE
   * \brief  we need to add the address of namespace device connected through 
   * wired (CSMA) connection to the UE because in LENA, in the downlink, the 
   * PGW uses the destination IP address to identify a single UE and the eNB 
   * to which it is attached to. 
   * If you specify a destination IP address that does not belong to any UE, 
   * the packet will just be dropped by the PGW, because it 
   * would not know to which eNB it should be routed through. This function
   * aims to resolve this problem. 
   * \param pgw PGW node 
   * \param ueLteNetDev lteNetDevice installed on the UE (nedded to get IMSI)
   * \param ueAddr the IPv4 address of the LXC container connected to UE
*/
void
addBackAddress (Ptr<Node> pgw, Ptr<NetDevice> ueLteNetDev, Ipv4Address addr)
{
  // First we need to get IMSI of ueLteNetDevice 
  Ptr<LteUeNetDevice> uedev = ueLteNetDev->GetObject<LteUeNetDevice> ();
  uint64_t imsi = uedev->GetImsi ();

  // get PGW application from pgw node
  Ptr<EpcPgwApplication> pgwApp = 
                    pgw->GetApplication (0)->GetObject<EpcPgwApplication>();

  // add container address to allow traffic through PGW node
  // drawback: no traffic can be sent to UE divice dirrectly 
  pgwApp->SetUeAddress (imsi, addr);
}

/**
 * \brief Calculate and print realtime execution jitter time, i.e differance 
 * between host wall clock and simulation clock.  
 * \param rt RealtimeSimulatorImpl pointer 
 * TODO :: add period param for scheduling (now is 100 ms by def)   
 */
void
JitterMonitor (Ptr<RealtimeSimulatorImpl> rt)
{

  Time mc = rt->Now ();
  Time ts = rt->RealtimeNow ();
  Time j = (ts >= mc) ? ts - mc : mc - ts;
  int64_t jitter = j.GetMilliSeconds(); 
  if (jitter > MAX_JITTER){
      std::cout << "! JITTER ! : " << jitter << std::endl;
  }
  Simulator::Schedule (MilliSeconds (500), &JitterMonitor, rt);
}

/**
 * @brief Generate UDP beacons on RSU if CAM service is disabled 
 * 
 * @param socket broadcast socket 
 * @param pktInterval interval im ms (default 100?)
 */

static void 
GenerateUdpBeacons (Ptr<Socket> socket, Time pktInterval )
{
  uint32_t pktSize = 10; // bytes  (+64 bytes)   
  socket->Send (Create<Packet> (pktSize));
  Simulator::Schedule (pktInterval, &GenerateUdpBeacons, socket, pktInterval);
}

/**
 * @brief get and print position of our vehicle periodicaly
 * 
 * @param sumoClient 
 * @param vId 
 */

void
PrintVehicleData ( Ptr<TraciClient> sumoClient, const std::string vId){
  libsumo::TraCIPosition traPos = sumoClient->TraCIAPI::vehicle.getPosition(vId); 
  double vSpeed = sumoClient->TraCIAPI::vehicle.getSpeed(vId); 
  std::cout <<  Simulator::Now().GetSeconds() << " , "
            << traPos.getString() 
            << ", speed: " << vSpeed
            << ", RSSI dBm: " << g_signalDbm
            << std::endl;

  Simulator::Schedule (Seconds(1), &PrintVehicleData, sumoClient, vId); 
}

/**
 * @brief Change periodicaly max vehicle speed. Range [10 - 100] km/h
 * change speed every [1 - 2]  min    
 * @param sumoClient 
 * @param vehId 
 */

void ChangeVehicleSpeed (Ptr<TraciClient> sumoClient , std::string vehId, uint32_t interval){
  
  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  x->SetAttribute ("Min", DoubleValue (MIN_VEH_SPEED));
  x->SetAttribute ("Max", DoubleValue (MAX_VEH_SPEED));
  double newMaxSpeed = x->GetValue();  
  
  std::cout << "New " << vehId 
            << "  max speed:"
            << newMaxSpeed << " km/h" << std::endl;
  
  sumoClient->TraCIAPI::vehicle.setMaxSpeed(vehId, newMaxSpeed/3.6); 

  // TODO: refactor rand  
  Simulator::Schedule (Seconds(interval), 
                        &ChangeVehicleSpeed, sumoClient, vehId, interval);

}


/**
 * \brief Read from the mob_trace the number of vehicles that will be created.
  *The number of vehicles is directly parsed from the rou.xml file, looking at 
  * all the valid XML elements of type <vehicle
  *\param path to rou.xml file to be parsed  
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
      NS_FATAL_ERROR ("Error: unable to parse the specified XML file: "<< path);
    }
  int numberOfNodes = -1;
  numberOfNodes = XML_rou_count_vehicles (rou_xml_file);
  xmlFreeDoc (rou_xml_file);
  xmlCleanupParser ();
  if (numberOfNodes == -1)
    {
      NS_FATAL_ERROR(
            "Fatal error: cannot gather the number of vehicles from the \
            specified XML file:"
            << path << ". Please check if it is a correct SUMO file.");
    }
  return numberOfNodes;
}


/**
 * @brief open in write mode, non blocking (O_NDELAY) This will cause open() 
 * to return -1 if there are no processes that have the file open for reading.
 * close silulation if no process are listening.. 
 * Note: we should run ns3 with sudo , since netns run under root and permitions
 * of pipe
 */
void 
mptcpd_open_pipe (){
    
    // close fd if exists one 
    if (mptcpdFd > 0) {
        close (mptcpdFd);
        mptcpdFd = -1;  
    }

    // open new fd  
    if (mptcpdFd < 0 ){
        std::cout << "NEW pipe connection:" << SSPI_FIFO_PATH << std::endl; 
        if ((mptcpdFd = open (SSPI_FIFO_PATH, O_WRONLY, O_NDELAY)) < 0 ){
            NS_FATAL_ERROR ("Error: unable to open pipe: " << SSPI_FIFO_PATH);
        } 
    } 
}

/**
 * @brief send command to userspace mptcp path manager  
 * @param msg @c struct sspi_cmd_message to send to our plugin through FIFO
 */
void
mptcpd_cmd_write (struct sspi_cmd_message const *msg)
{
    // connect to pipe if first time called
    if (mptcpdFd < 0)
        mptcpd_open_pipe ();
    
    std::cout << "cmd: " << (int)msg->cmd << " val: " << msg->cmd_value << std::endl; 
    
    // create command message end send it
    struct sspi_message c_msg = {.type = SSPI_MSG_TYPE_CMD};
    std::memcpy (c_msg.data, msg, sizeof (sspi_cmd_message));    
    
    if (write (mptcpdFd, &c_msg, sizeof (c_msg)) < 0) {
            std::cout << "Can't write command to pipe \n";
            mptcpdFd = -1; // try to reconect on next time
    }
}

/**
 * @brief send data message to userspace mptcp path manager 
 * @param msg @c struct sspi_data_message to send 
 */

void 
mptcpd_data_write (struct sspi_data_message const *msg){
    
    // connect to pipe if first time called
    if (mptcpdFd < 0)
        mptcpd_open_pipe ();

    // create message end send it
    struct sspi_message d_msg = {.type = SSPI_MSG_TYPE_DATA};
    std::memcpy (d_msg.data, msg, sizeof (sspi_data_message));
    
    if (write (mptcpdFd, &d_msg, sizeof (d_msg)) < 0) {
        std::cout << "Can't write data to pipe \n";
        mptcpdFd = -1; // try to reconect on next time
    }
}

/**
 * @brief send cmd to start tcpdump on all inetrfaces 
 * @param time duration when send SIGKILL for tcpdump process
 */
void RunTcpdump ( int time){
     struct sspi_cmd_message msg = {.cmd = SSPI_CMD_TCPDUMP, 
                                            .cmd_value = time};
    mptcpd_cmd_write (&msg);                                             
}

/**
 * @brief send command to plugin to generate traffic with iperf 
 * @param duration time to run (-t)
 * @param repeat  continue to generate traffic with iperf until simulation ends,
 * iperf session duration is randomized, between [10:100] seconds = 
 * (+-) 5:50 MB file size
 */
void RunIperf ( int duration, bool repeat)
{
    // set session duration, be cearful to not exceed total simTime
    double _now = Simulator::Now().GetSeconds(); 
    if (duration + _now + 1 >= simTime){
        duration = simTime - _now - 1; 
    } 
    if (duration < 0) return;

    struct sspi_cmd_message sspi_msg = {.cmd = SSPI_CMD_IPERF_START, 
                                        .cmd_value = (int) duration};
    mptcpd_cmd_write (&sspi_msg);
    std::cout << "New iperf session, duration: " << duration << " sec" << std::endl;

    if (repeat){
        // set random next session duration
        Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
        x->SetAttribute ("Min", DoubleValue (10));
        x->SetAttribute ("Max", DoubleValue (100));
        Simulator::Schedule (Seconds (duration + 1), &RunIperf, 
                                            (int) x->GetInteger(), repeat);
        } 
}


/**
   * \brief TracedCallback signature for monitor mode receive events.
   *
   * \param packet the packet being received
   * \param channelFreqMhz the frequency in MHz at which the packet is
   *        received. Note that in real devices this is normally the
   *        frequency to which  the receiver is tuned, and this can be
   *        different than the frequency at which the packet was originally
   *        transmitted. This is because it is possible to have the receiver
   *        tuned on a given channel and still to be able to receive packets
   *        on a nearby channel.
   * \param txVector the TXVECTOR that holds RX parameters
   * \param aMpdu the type of the packet (0 is not A-MPDU, 1 is a MPDU that 
   *        is part of an A-MPDU and 2 is the last MPDU in an A-MPDU)
   *        and the A-MPDU reference number (must be a different value for 
   *        each A-MPDU but the same for each subframe within one A-MPDU)
   * \param signalNoise signal power and noise power in dBm
   * \param staId the STA-ID  (always 65535)
   * \todo WifiTxVector should be passed by const reference because
   * of its size.
   */
void
MonitorSniffRx (Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector,
                MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId)

{
    //   g_samples++;
    //   g_signalDbmAvg += ((signalNoise.signal - g_signalDbmAvg) / g_samples);
    //   g_noiseDbmAvg += ((signalNoise.noise - g_noiseDbmAvg) / g_samples);

    // //   staId == 65535
    // //   channelFreqMhz ==  5860
    //   std::cout << Simulator::Now().GetSeconds()  <<"\t" << signalNoise.signal
    //                                               <<"\t" << signalNoise.noise
    //                                               << std::endl ;
    // 34.5908	-81.7451	-96.9763
    // send backup flag (initially sspi:backup = false)

    // Beacon size == 74 
    // std::cout << " Packet size : " << packet->GetSize() << std::endl; 
    // SIMPLE LPF  α * x[i] + (1-α) * y[i-1]
    //  double RSSI_T = -80; // RSSI Threashold value
    
    double alpha = 0.85; // alpha value 
    g_signalDbm = alpha * signalNoise.signal + (1-alpha)*g_signalDbm; 
    
    // prepare data msg to send 
    struct sspi_data_message msg_data = {.rss = signalNoise.signal, 
                                            .noise = signalNoise.noise }; 

    NS_UNUSED(msg_data); 
    //mptcpd_data_write (&msg_data); 

    //   Ipv4Header ipv4H ;
    //   WifiMacHeader wifiH;
    //   if (uint32_t num = packet->PeekHeader(wifiH)){
    //     std::cout<< "num : "<<num << "\ta1:"<<  wifiH.GetAddr1()
    //                 << "\ta2:"<<  wifiH.GetAddr2() << std::endl;
    //   }
}

// ***************************************************************************
// ***************************************************************************
//                                      MAIN
// ***************************************************************************
// ***************************************************************************
int
main (int argc, char *argv[])
{

  // ****************************************
  // Global configurations
  // ****************************************

  bool sumo_gui = false;        // no SUMO GUI by defauls 
  double sumo_updates = 0.01;   // sumo update rate 
  std::string csv_name;
  bool verbose = true ;
  double maxVehSpeed = 25.0 ;     // initial max veh speed [km/h]
  uint32_t changeVehMaxSpeed = 0; // change periodicaly veh max speed, the interval in ms
  bool iperfReapeat = false ;     // repeat iperf sessions  
  bool startTcpdump = false;      // capture traffic on namespace 

  /* MPTCP NETLINK PATH MANAGER */
  bool use_mptcp_pm = false ;     // use mptcpd path manager or not
  int iperf_duration = 0;         // start iperf session time is sec 
  int iperfStart = 1 ;            // start generate traffic from time  
  mptcpdFd = -1 ;                 // file desc to connect to mptcpd plugin
  g_signalDbm = -85;              // init value rssi (dBm)
  average_RSU_conn_time = 0.0;    // to calculate av connection time on RSU
  
  /*****  MS-VAN3T configuration *****/
  std::string m_vId = "veh1";     // sumo id of our first vehicle
  bool send_cam = true;           // enable CAM service on vehicles and RSU
  
  // double m_baseline_prr = 150.0;
  // bool m_prr_sup = false;
  //bool print_summary = false; // To print summary at the end of simulation

  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config = 
                "src/automotive/examples/sumo_files_v2v_map/map.sumo.cfg";

  //
  // Allow the user to override any of the defaults at run-time, via 
  // command-line arguments
  //
  CommandLine cmd (__FILE__);
  cmd.AddValue ("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("use-mptcpd", 
                        "Use or not netlink MPTCP Path manager", use_mptcp_pm); 
  cmd.AddValue ("iperf", 
                "iperf session duration", iperf_duration); 
  cmd.AddValue ("iperfStart", 
                "Start generate traffic from time [s],", iperfStart); 
  cmd.AddValue ("tcpdump", 
                "Capture traffic on interfaces of namespace during simTime-1", 
                        startTcpdump); 
  cmd.AddValue ("verbose", "Print debug data every second", verbose);
  cmd.AddValue ("iperfRepeat", "Repeat iperf sessions umtil simulation end, the time of sessio is random value [10:100]sec", iperfReapeat); 
  cmd.AddValue ("changeVehMaxSpeed", "The interval in [s] to change periodicaly vehicles max speed randomly from [40:120]km/h don\'t change if 0", changeVehMaxSpeed); 
  cmd.AddValue ("maxSpeed", "initial max veh speed [km/h]", maxVehSpeed); 
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", 
                                                 sumo_config);
  cmd.AddValue ("send-cam",
                "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",  send_cam);

  // cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  // cmd.AddValue ("summary",
  //               "Print a summary for each vehicle at the end of the simulation",
  //               print_summary);
  //  cmd.AddValue ("vehicle-visualizer",
  //  "Activate the web-based vehicle visualizer for ms-van3t",
  //  vehicle_vis);

  //   cmd.AddValue ("csv-log-cumulative",
  //                     "Name of the CSV log file for the cumulative (average) 
  //                     PRR and latency data",
  //                     csv_name_cumulative);

  // cmd.AddValue ("netstate-dump-file",
  //     "Name of the SUMO netstate-dump file containing the vehicle-related 
  //      information throughout the whole simulation", sumo_netstate_file_name);
  // cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  // cmd.AddValue ("prr-sup", "Use the PRR supervisor or not", m_prr_sup);

  cmd.Parse (argc, argv);

  // @TODO check some input param errors, make this better and informative 
  if (simTime < iperfStart) iperfStart = 0 ;  
  if (simTime < iperf_duration) iperf_duration = 0 ;

  // Turn on simulation in real-time mode  
  GlobalValue::Bind ("SimulatorImplementationType", 
                    StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
  // --ns3::RealtimeSimulatorImpl::SynchronizationMode=HardLimit

  // get pointer to real-time simulation impl
  ns3::Ptr<ns3::RealtimeSimulatorImpl> realSim =
                    ns3::DynamicCast<ns3::RealtimeSimulatorImpl> 
                                    (ns3::Simulator::GetImplementation ());

  //
  // ****************************************
  //            Nodes creation
  // ****************************************
  // Create two ghost nodes.  The first will represent the virtual machine host
  // on the left side of the network; and the second will represent the VM on
  // the right side.
  //

  NodeContainer nodes;      // Create two ghost nodes.
  nodes.Create (2);

  NodeContainer nodeAP;     // create one RSU node
  nodeAP.Create (1);

  NodeContainer enbNode;    // create one eNB node
  enbNode.Create (1);

  int numberOfNodes;        // number of vehicles from rou.xml file
  NodeContainer obuNodes;   // node container for vehicle OBU
  uint32_t nodeCounter = 0; //

  // ****************************************
  // Helpers used in simulation
  // ****************************************
  InternetStackHelper inet; // internet stack helper
  CsmaHelper csma;          // Csma Helper
  Ipv4AddressHelper ipv4h;  // Ipv4 Address Helper
  Ipv4StaticRoutingHelper ipv4RoutingHelper; // Ipv4 Static Routing Helper

  // ****************************************
  // Add a default internet stack to the node (ARP, IPv4, ICMP, UDP and TCP).
  // ****************************************
  inet.Install (nodeAP);
  inet.Install (nodes);
 

  // **************************************************************************
  //                      MOBILITY MODEL
  // **************************************************************************
  Ptr<ListPositionAllocator> positionAlloc = 
                            CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0, 0, 0)); // pos of left
  positionAlloc->Add (Vector (25, -50, 10)); // pos of AP  --> center of SUMO map
  positionAlloc->Add (Vector (25, 0, 0)); // pos of right
  positionAlloc->Add (Vector (60, -50, 20)); // pos of eNodeB

  MobilityHelper mobility; // mobility helper
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes.Get (0));
  mobility.Install (nodeAP.Get (0));
  mobility.Install (nodes.Get (1));
  mobility.Install (enbNode.Get (0));

  // **************************************************************************
  //                     Configure PATH 1: (802.11p and CSMA)
  // **************************************************************************

  NodeContainer nodes_l_ap (nodes.Get (0), nodeAP.Get (0));
  NodeContainer nodes_r_ap (nodes.Get (1), nodeAP.Get (0));

  /* 802.11p param configuration */
  int txPower = 23;
  float datarate = 6;
  std::string phyMode ("OfdmRate6MbpsBW10MHz");

  /*** 2. Create and setup channel   ***/
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  
  std::cout << "Setting up the 802.11p channel @ " 
            << datarate << " Mbit/s, 10 MHz, and tx power "
            << (int) txPower << " dBm." << std::endl;

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);

  /*** 3. Create and setup MAC ***/
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager", 
                                      "DataMode", StringValue (phyMode), 
                                      "ControlMode", StringValue (phyMode));
  
  // NetDeviceContainer dev_wifi_l_ap = 
  //               wifi80211p.Install (wifiPhy, wifi80211pMac, nodes_l_ap);
  
  // Install devices on nodes from path #1
  NetDeviceContainer dev_l_ap = 
                wifi80211p.Install (wifiPhy, wifi80211pMac, nodes_l_ap);

  // index of interfaces to use in CAM service 
  uint32_t ifIdxVeh =  dev_l_ap.Get(0)->GetIfIndex();
  uint32_t ifIdxRsu =  dev_l_ap.Get(1)->GetIfIndex();

  // configure CSMA AP <--> REMOTE
  csma.SetChannelAttribute ("DataRate", StringValue ("10Gbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (1)));
  NetDeviceContainer dev_r_ap = csma.Install (nodes_r_ap);

  // Assign adress
  ipv4h.SetBase ("16.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_ap_wifi = ipv4h.Assign (dev_l_ap.Get (1));
  Ipv4InterfaceContainer ifce_l_wifi = ipv4h.Assign (dev_l_ap.Get (0));
  ipv4h.SetBase ("17.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_ap_csma = ipv4h.Assign (dev_r_ap.Get (1));
  Ipv4InterfaceContainer ifce_r_csma = ipv4h.Assign (dev_r_ap.Get (0));

  // ************************************************************************
  //                 Configure PATH 2: LTE and CSMA
  // ************************************************************************

  // Default LTE configuration
  Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", 
                                                BooleanValue (false));
  Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", 
                                                BooleanValue (false));
  Config::SetDefault ("ns3::RrFfMacScheduler::HarqEnabled", 
                                                BooleanValue (false));
  Config::SetDefault ("ns3::LteHelper::PathlossModel",
                      StringValue ("ns3::FriisSpectrumPropagationLossModel"));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth",
                        UintegerValue (100)); // 5MHz,  100 for 20MHz bandwidth
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", 
                        UintegerValue (100)); //20MHz bandwidth
//   bool useCa = true; 
//   if (useCa)
//     {
//       Config::SetDefault ("ns3::LteHelper::UseCa", BooleanValue (useCa));
//       Config::SetDefault ("ns3::LteHelper::NumberOfComponentCarriers",
//                                         UintegerValue (2));
//       Config::SetDefault ("ns3::LteHelper::EnbComponentCarrierManager",
//                           StringValue ("ns3::RrComponentCarrierManager"));
//     }

  // get LTE / EPC helpers
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = 
                            CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  // Get the pgw node to install the csma device
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Install LTE Devices to the nodes
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNode);
  // TODO: test without NodeContainer
  NetDeviceContainer ueLteDevs = 
        lteHelper->InstallUeDevice (NodeContainer (nodes.Get (0))); 
  
  // assign ipv4 adresses to UE Devices
  Ipv4InterfaceContainer ifce_l_lte = 
                    epcHelper->AssignUeIpv4Address (ueLteDevs);

  // Attach one UE per eNodeB
  // side effect: the default EPS bearer will be activated
  lteHelper->Attach (ueLteDevs.Get (0), enbLteDevs.Get (0));

  // Link: PGW <---> Remote node through CSMA
  csma.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("10Gb/s")));
  csma.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (1)));
  NodeContainer nodes_r_pgw (nodes.Get (1), pgw);
  NetDeviceContainer dev_r_pgw = csma.Install (nodes_r_pgw);

  // and assign ipv4 adress to ifaces
  ipv4h.SetBase ("12.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_pgw_csma = 
                    ipv4h.Assign (NetDeviceContainer (dev_r_pgw.Get (1)));
  Ipv4InterfaceContainer ifce_r_csma1 = 
                    ipv4h.Assign (NetDeviceContainer (dev_r_pgw.Get (0)));

  
  // *************************************************************************
  //               Setup TRACI/SUMO and configure other Vehicles
  // *************************************************************************

  /*** 0.a App Options ***/
  
  // 0.b parsing xml to get num of veh
  std::cout << "Reading the .rou file..." << std::endl;
  std::string path = sumo_folder + mob_trace;
  numberOfNodes = getNumberVeh (path); // parsing sumo xml to get num vehicles
  std::cout << "The .rou file has been read: " << numberOfNodes
            << "vehicles will be present in the simulation." << std::endl;

  /* 1. Configure dynamic vehicles */
  //(-1) since our UE node is placed in the first vehicle
  obuNodes.Create (numberOfNodes - 1); 
  if (obuNodes.GetN () > 0)
    {
      mobility.Install (obuNodes); // install mobility model

      //install 802.11p devices
      NetDeviceContainer obuNetDevices =
          wifi80211p.Install (wifiPhy, wifi80211pMac, obuNodes); 
      
      /* Give packet socket powers to nodes (otherwise, 
        if the app tries to create a PacketSocket, 
        CreateSocket will end up with a segmentation fault */
      PacketSocketHelper packetSocket;
      packetSocket.Install (obuNodes);

      // OR remove 2 lines above and uncomment this lines  
      // inet.Install (obuNodes);
    }

  /*** 2. Configure sumo cliente  ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  
  // use system installation of sumo
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue ("")); 
  
  sumoClient->SetAttribute ("SynchInterval", 
                            TimeValue (Seconds (sumo_updates)));
  
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", (BooleanValue) sumo_gui);
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));

    // SUMO options on start 
  std::string sumo_additional_options = "--verbose true \
                            --collision.action warn \
                            --collision.check-junctions \
                            --error-log=sumo-errors-or-collisions.xml";

  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));
  
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", 
                        StringValue (sumo_additional_options));

  /***  Setup and start CA service on RSU  *****/
  if (send_cam) {
    simpleCAMSenderHelper SimpleCAMSenderHelper;
    SimpleCAMSenderHelper.SetAttribute ("RealTime", BooleanValue (true));
    SimpleCAMSenderHelper.SetAttribute ("Client", (PointerValue) sumoClient);
    SimpleCAMSenderHelper.SetAttribute("IsRSU", BooleanValue(true));
    SimpleCAMSenderHelper.SetAttribute("Interface", IntegerValue(ifIdxRsu));
      
    ApplicationContainer simpleCamSenderApp = 
                                        SimpleCAMSenderHelper.Install (nodeAP);
    simpleCamSenderApp.Start (Seconds (0.0));
    simpleCamSenderApp.Stop (Seconds (simTime - 0.1) - Simulator::Now ());
  }

  /* callback function for node creation, return newlly created Node  */
  std::function<Ptr<Node> ()> setupNewWifiNode = [&] () -> Ptr<Node> {
    Ptr<Node> includedNode;

    /* first time call this function veh1 is our UE */
    if (nodeCounter == 0)
      {
        includedNode = nodes.Get (0);

        // set different color to our vehicle
        libsumo::TraCIColor red;
        red.r = 255; red.g = 0; red.b = 0; red.a = 255;
        sumoClient->TraCIAPI::vehicle.setColor (m_vId, red);
        sumoClient->TraCIAPI::vehicle.setMaxSpeed (m_vId, maxVehSpeed / 3.6);
      }
    else
      {
        // get from the node pool (starting from 0)
        includedNode = obuNodes.Get (nodeCounter - 1);
      }

    /***  Setup and start CA service on Vehicles  *****/
    if (send_cam)
      {
        simpleCAMSenderHelper SimpleCAMHelper;
        SimpleCAMHelper.SetAttribute ("RealTime", BooleanValue (true));
        SimpleCAMHelper.SetAttribute ("Client", (PointerValue) sumoClient);
        if (nodeCounter == 0)
          SimpleCAMHelper.SetAttribute ("Interface", IntegerValue (ifIdxVeh));
        
        ApplicationContainer setupAppSimpleSender = 
                                        SimpleCAMHelper.Install (includedNode);
        setupAppSimpleSender.Start (Seconds (0.0));
        setupAppSimpleSender.Stop (Seconds (simTime - 0.1) - Simulator::Now());
      }

    std::cout << "Created Vehicle ID : " << nodeCounter << ", Node ID : " << includedNode->GetId ()
              << std::endl;

    nodeCounter++; //
    return includedNode;
  };

  /* callback function for node shutdown */
  std::function<void (Ptr<Node>)> shutdownWifiNode = [] (Ptr<Node> exNode) {
    
    /* Stop all applications right now : 1ms sheduler */
    for (uint32_t app =0 ; app < exNode->GetNApplications(); app ++){
        exNode->GetApplication(app)->SetStopTime(MilliSeconds(1)); 
    } 

    // Ptr<areaSpeedAdvisorClient80211p> areaSpeedAdvisorClient80211p_ =
    //     exNode->GetApplication (0)->GetObject<areaSpeedAdvisorClient80211p> ();
    // if (areaSpeedAdvisorClient80211p_)
    //   areaSpeedAdvisorClient80211p_->StopApplicationNow ();

    /* Set position outside communication range */
    Ptr<ConstantPositionMobilityModel> mob = 
                exNode->GetObject<ConstantPositionMobilityModel> ();
    mob->SetPosition (Vector (-1000.0 + (rand () % 25), 320.0 + (rand () % 25),
                              250.0)); // rand() for visualization purposes

    /* NOTE: further actions could be required for a safe shut down! */
  };

  /* Start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  // **********************************************************************
  //    Configure FDNetDevices and connect to TAP file descriptor
  // **********************************************************************

  EmuFdNetDeviceHelper fdNet;
  NetDeviceContainer devices;
  std::string encapMode ("Dix");
  fdNet.SetAttribute ("EncapsulationMode", StringValue (encapMode));

  fdNet.SetDeviceName ("vethLeft");
  devices = fdNet.Install (nodes.Get (0)); 
  Ptr<NetDevice> leftFdDev = devices.Get (0);
  leftFdDev->SetAttribute (
                "Address", Mac48AddressValue (Mac48Address::Allocate ()));

  fdNet.SetDeviceName ("vethRight");
  devices = fdNet.Install (nodes.Get (1));
  Ptr<NetDevice> rightFdDev = devices.Get (0);
  rightFdDev->SetAttribute (
                "Address", Mac48AddressValue (Mac48Address::Allocate ()));

  fdNet.SetDeviceName ("vethLeft1");
  devices = fdNet.Install (nodes.Get (0));
  Ptr<NetDevice> leftFdDev1 = devices.Get (0);
  leftFdDev1->SetAttribute (
                "Address", Mac48AddressValue (Mac48Address::Allocate ()));

  fdNet.SetDeviceName ("vethRight1");
  devices = fdNet.Install (nodes.Get (1));
  Ptr<NetDevice> rightFdDev1 = devices.Get (0);
  rightFdDev1->SetAttribute (
                "Address", Mac48AddressValue (Mac48Address::Allocate ()));

  /*
    we have a FdNetDevices connected to veth devices ,
    now install it on nodes and set IP addresses
  */
  Ptr<Ipv4> ipv4;               // get Ipv4 object from node
  Ipv4InterfaceAddress address; // interface address
  uint32_t interface;           // iface number

  // left node (UE) 1st inface
  ipv4 = nodes.Get (0)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (leftFdDev1);
  address = Ipv4InterfaceAddress (
                Ipv4Address ("15.0.0.1"), Ipv4Mask ("255.0.0.0"));
  ipv4->AddAddress (interface, address);
  //ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  // left node (UE) 2nd inface 
  ipv4 = nodes.Get (0)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (leftFdDev);
  address = Ipv4InterfaceAddress (
                Ipv4Address ("11.0.0.1"), Ipv4Mask ("255.0.0.0"));
  ipv4->AddAddress (interface, address);
  ipv4->SetUp (interface);

  // right node (Remote) 1st inface
  ipv4 = nodes.Get (1)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (rightFdDev1);
  address = Ipv4InterfaceAddress (
                Ipv4Address ("14.0.0.1"), Ipv4Mask ("255.0.0.0"));
  ipv4->AddAddress (interface, address);
  ipv4->SetUp (interface);

  // right node (Remote) 2nd inface 
  ipv4 = nodes.Get (1)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (rightFdDev);
  address = Ipv4InterfaceAddress (
                Ipv4Address ("13.0.0.1"), Ipv4Mask ("255.0.0.0"));
  ipv4->AddAddress (interface, address);
  ipv4->SetUp (interface);

  // *************************************************************************
  //                        Configure ROUTING
  // *************************************************************************

  //Adding the network behind the UE and remote host to the AP routing table
  // hardcoding the IP address of the LXC interfaces connected to this path
  // TODO :::: change routing

  // *********************************
  // routing on AP node
  // *********************************
  Ptr<Ipv4StaticRouting> apStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (nodeAP.Get (0)->GetObject<Ipv4> ());
  apStaticRouting->AddNetworkRouteTo (
                Ipv4Address ("15.0.0.0"), Ipv4Mask ("255.0.0.0"),
                Ipv4Address ("16.0.0.2"), ifce_ap_wifi.Get (0).second);
  
  apStaticRouting->AddNetworkRouteTo (
                Ipv4Address ("11.0.0.0"), Ipv4Mask ("255.0.0.0"),
                Ipv4Address ("16.0.0.2"), ifce_ap_wifi.Get (0).second);
  
  apStaticRouting->AddNetworkRouteTo (
                Ipv4Address ("14.0.0.0"), Ipv4Mask ("255.0.0.0"),
                Ipv4Address ("17.0.0.2"), ifce_ap_csma.Get (0).second);
  
  apStaticRouting->AddNetworkRouteTo (
                Ipv4Address ("13.0.0.0"), Ipv4Mask ("255.0.0.0"),
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
  ueStaticRouting->AddNetworkRouteTo (
                Ipv4Address ("14.0.0.0"), Ipv4Mask ("255.0.0.0"),
                Ipv4Address ("16.0.0.1"), ifce_l_wifi.Get (0).second);

  // ********************************
  // routing on right node (Remote)
  // ********************************
  Ptr<Ipv4StaticRouting> rightStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (nodes.Get (1)->GetObject<Ipv4> ());
  
  // default route through LTE path
  rightStaticRouting->SetDefaultRoute (
                Ipv4Address ("12.0.0.1"), ifce_r_csma1.Get (0).second);
  
  // add route to left lxc through wifi
  rightStaticRouting->AddNetworkRouteTo (
                Ipv4Address ("15.0.0.0"), Ipv4Mask ("255.0.0.0"),
                Ipv4Address ("17.0.0.1"), ifce_r_csma.Get (0).second);
  
  // add route to left lxc through wifi
  // do non need route to 16. just for ping test
  rightStaticRouting->AddNetworkRouteTo (
                Ipv4Address ("16.0.0.0"), Ipv4Mask ("255.0.0.0"),
                Ipv4Address ("17.0.0.1"), ifce_r_csma.Get (0).second);

  // ********************************
  // routing on PGW node
  // ********************************
  // Adding the network behind the UE to the pgw
  // hardcoding the IP address of the UE connected to the external network
  Ptr<Ipv4StaticRouting> pgwStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (pgw->GetObject<Ipv4> ());
  
  pgwStaticRouting->AddNetworkRouteTo (
                Ipv4Address ("11.0.0.0"), Ipv4Mask ("255.0.0.0"),
                Ipv4Address ("7.0.0.2"), 1);

  pgwStaticRouting->AddNetworkRouteTo (
                Ipv4Address ("13.0.0.0"), Ipv4Mask ("255.0.0.0"),
                Ipv4Address ("12.0.0.2"), ifce_pgw_csma.Get (0).second);

  // Add LXC left address to allow connection through LTE network (EPC-PGW)
  addBackAddress (pgw, ueLteDevs.Get (0), Ipv4Address ("11.0.0.2"));

  // **************************************************************************
  // **************************************************************************
  //                        Configure APPLICATIONS
  // **************************************************************************
  // **************************************************************************
  

  /* Broadcast UDP packets by RSU if CAM service is disabled*/ 

  if (!send_cam){

      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      Ptr<Socket> source = Socket::CreateSocket (nodeAP.Get (0), tid);
      // broadcast beacons 
      InetSocketAddress remote = 
              InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
      source->SetAllowBroadcast (true);
      source->Connect (remote);

      // milliseconds send pack interval 100 ms
      Time inter_beacon_interval = MilliSeconds (100.0); 
      Simulator::ScheduleWithContext (source->GetNode ()->GetId (), 
                                      Seconds (1.0), &GenerateUdpBeacons,
                                      source, 
                                      inter_beacon_interval);
  }

  // ********************************************************
  // Debug: Testing that proper IP addresses are configured
  // ********************************************************
  //wifiPhy.EnablePcap("mp-v2i",nodes);  
  //wifiPhy.EnablePcap("mp-v2i",nodeAP);  
  //wifiPhy.EnablePcap("mp-v2i",obuNodes);  
  //wifiPhy.EnablePcapAll ("mp-wifi", true);
  // csma.EnablePcapAll ("mp-csma", true);
  // fdNet.EnablePcapAll ("mp-fd", true);

  // realtime jitter calculation :
  // change interval
  Simulator::Schedule (MilliSeconds(100), &JitterMonitor, realSim);
  

  // ********************************************************
  // MPTCPD: Interaction with MPTCPD Path manager
  // ********************************************************
  
  // collect pcap data durring the simulation with TcpDump  
  if (startTcpdump){
      Simulator::Schedule (MilliSeconds (500), &RunTcpdump, simTime-1);
  }

  // use MPTCPD PM to control subflows
  // monitor 802.11p RF metrics for each received pack and sends commands 
  // to set BACKUP flag
  if (use_mptcp_pm){
      Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Phy/MonitorSnifferRx",
                                     MakeCallback (&MonitorSniffRx));
  }

    /**
    * generate traffic with iperf  
    */
  if (iperf_duration){
        Simulator::Schedule (Seconds(iperfStart), &RunIperf, 
                                            iperf_duration, iperfReapeat);
    }
  


  // Print debug info 
  if (verbose){
      Simulator::Schedule ( MilliSeconds (1000), 
                            &PrintVehicleData, sumoClient, m_vId);
  }

  // Update vehicle max speed to simulate different traffic patterns 
  if (changeVehMaxSpeed){
    Simulator::Schedule ( Seconds (changeVehMaxSpeed), 
                          &ChangeVehicleSpeed, sumoClient, m_vId, 
                          changeVehMaxSpeed);
  }

  //Simulator::Schedule (Seconds(iperfStart+2), &set_endpoin_backup, SSPI_IFACE_LTE);
   
  // Show parameters :
   std::stringstream conf_param; 
   conf_param   << "SimTime: "          << simTime << "\n" 
                << "TcpDump: "          << startTcpdump << "\n" 
                << "IperfTime: "        << iperf_duration << "\n" 
                << "IperfStart: "       << iperfStart << "\n" 
                << "Verbose: "          << verbose << "\n"
                << "Max Speed: "        << maxVehSpeed << "\n"
                << "Change Speed: "     << changeVehMaxSpeed << "\n" 
                << "Repeat iperf: "     << iperfReapeat << "\n"
                << "Enable CAM: "       << send_cam << "\n";  
    
    if (!send_cam) conf_param << "\t CA service Disabled: UDP traffic will be generated";

    // show configuration params; 
    std::cout << conf_param.str() << std::endl;

    /*
        config output , write ns-3 config params to file
    */
    Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
    Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
    Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
    ConfigStore outputConfig;
    outputConfig.ConfigureDefaults ();
    outputConfig.ConfigureAttributes ();

    /*** 10. Setup and Start Simulation + Animation ***/
    // AnimationInterface anim ("ns3-sumo-coupling.xml"); // Mandatory
    // Now, do the actual emulation.
    //
    NS_LOG_INFO ("Run Emulation.");
    Simulator::Stop (Seconds (simTime));
    Simulator::Run ();
    Simulator::Destroy ();
    NS_LOG_INFO ("Done.");
}
