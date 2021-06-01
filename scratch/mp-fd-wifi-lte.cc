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

#include <fcntl.h>  /* O_RDWR */
#include <string.h> /* memset(), memcpy() */
#include <stdio.h> /* perror(), printf(), fprintf() */
#include <stdlib.h> /* exit(), malloc(), free() */
#include <sys/ioctl.h> /* ioctl() */

/* includes for struct ifreq, etc */
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <unistd.h>

/**
   * \brief  we need file descriptor of previosly created TAP device to connect it to FdNetDevice
   * by using a function SetFileDescriptor(int fd). this way we can send and receive L2 trafic to/from 
   * real world  
   * \param dev device name in string (ex. "tap0")
   * \return int value : file descriptor of TAP device 
*/
int tun_open( std::string dev)
{
 
  struct ifreq ifr;
  int fd, err;
  const std::string clonedev = "/dev/net/tun";

   /* open the clone device */
   if( (fd = open(clonedev.c_str(), O_RDWR)) < 0 ) {
     return fd;
   }

   /* preparation of the struct ifr, of type "struct ifreq" */
   memset(&ifr, 0, sizeof(ifr));

   NS_ASSERT_MSG (!dev.empty (), "TAP device name should be specifyed");
   if (!dev.empty ())
     {
       NS_ASSERT_MSG (dev.size () > IFNAMSIZ - 1, " TAP device name size is too big");
       strncpy (ifr.ifr_name, dev.c_str (), dev.size () + 1);
       /* TAP interface, No packet information */
       ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
     }

   /* try to create the device */
   if( (err = ioctl(fd, TUNSETIFF, (void *) &ifr)) < 0 ) {
     close(fd);
     perror("ioctl TUNSETIFF");close(fd);
     std::exit(1); 
     return err;
   }  
  
  std::cout << dev<< ":  FD = "<< fd << std::endl; 

  /* this is the special file descriptor that the caller will use to talk
   * with the virtual interface */
  return fd; 

}

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TAPPingExample");

static void
PingRtt (std::string context, Time rtt)
{
  NS_LOG_UNCOND ("Received Response with RTT = " << rtt);
}

// ****************************************************************************************************************
// ****************************************************************************************************************
//                                      MAIN
// ****************************************************************************************************************
// ****************************************************************************************************************
int
main (int argc, char *argv[])
{
  NS_LOG_INFO ("Ping Emulation Example with TAP");

  std::string remote ("15.0.0.2"); 
  std::string mask ("255.0.0.0");
  std::string pi ("no");
  
  // tap devices name (should be pre created) 
  std::string tap_l ("tap-left");
  std::string tap_l1 ("tap-left-1");
  std::string tap_r ("tap-right");
  std::string tap_r1 ("tap-right-1");
  //
  // Allow the user to override any of the defaults at run-time, via
  // command-line arguments
  //
  CommandLine cmd (__FILE__);
  cmd.AddValue ("remote", "Remote IP address (dotted decimal only please)", remote);
  cmd.AddValue ("tapMask", "Network mask for configure the TAP device (dotted decimal only please)", mask);
  cmd.AddValue ("modePi", "If 'yes' a PI header will be added to the traffic traversing the device(flag IFF_NOPI will be unset).", pi);
  cmd.Parse (argc, argv);

  Ipv4Address remoteIp (remote.c_str ());
  Ipv4Mask tapMask (mask.c_str ());

  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  //
  // Create two ghost nodes.  The first will represent the virtual machine host
  // on the left side of the network; and the second will represent the VM on 
  // the right side.
  //
  NodeContainer nodes;
  nodes.Create (2);

  NodeContainer nodeAP;
  nodeAP.Create(1);     // create wifi AP node

  NodeContainer nodes_l_ap (nodes.Get(0), nodeAP.Get(0));
  NodeContainer nodes_r_ap (nodes.Get(1), nodeAP.Get(0));

  // ****************************************
  // Helpers used in simulation 
  // ****************************************
  MobilityHelper mobility;                      // mobility helper 
  InternetStackHelper inet;                     // internet stack helper 
  YansWifiChannelHelper wifiChannel;            // Yans Wifi Channel Helper
  YansWifiPhyHelper wifiPhy;                    // Yans Wifi Phy Helper
  WifiMacHelper wifiMac;                        // Wifi Mac Helper
  WifiHelper wifi;                              // Wifi Helper
  CsmaHelper csma;                              // Csma Helper
  Ipv4AddressHelper ipv4h;                      // Ipv4 Address Helper
  Ipv4StaticRoutingHelper ipv4RoutingHelper;    // Ipv4 Static Routing Helper

  // ****************************************
  // set possition and mobility model
  // ****************************************
  Ptr<ListPositionAllocator> positionAllocWifi = CreateObject<ListPositionAllocator> ();
  positionAllocWifi->Add (Vector (0.0, 0.0, 0.0));
  positionAllocWifi->Add (Vector (5, 0, 0));
  positionAllocWifi->Add (Vector (15, 0, 0));
  mobility.SetPositionAllocator (positionAllocWifi);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes.Get (0));
  mobility.Install (nodeAP.Get (0));
  mobility.Install (nodes.Get(1));
 
  // ****************************************
  // Add a default internet stack to the node (ARP, IPv4, ICMP, UDP and TCP).
  // ****************************************
  inet.Install (nodeAP);
  inet.Install (nodes); 

  // ****************************************************************************************************************
  //                                  Configure PATH 1: WI-FI and CSMA 
  // ****************************************************************************************************************

  // create default  wifi  channel 
  wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);  // allow pcap traces 
  wifiPhy.SetChannel (wifiChannel.Create ());                         // create channel 

   // Add a mac and Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  
  // wifi helper set params 
  wifi.SetStandard (WIFI_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
                                StringValue ("OfdmRate54Mbps"), 
                                "ControlMode", StringValue ("OfdmRate24Mbps"));

  // configure CSMA connection  
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (1)));

  // Install devices on nodes from path #1
  NetDeviceContainer dev_l_ap = wifi.Install (wifiPhy, wifiMac, nodes_l_ap);
  NetDeviceContainer dev_r_ap = csma.Install (nodes_r_ap);
  
  // Assign adress 
  ipv4h.SetBase ("16.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_ap_wifi = ipv4h.Assign (dev_l_ap.Get(1));
  Ipv4InterfaceContainer ifce_l_wifi = ipv4h.Assign (dev_l_ap.Get(0));
  ipv4h.SetBase ("17.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_ap_csma = ipv4h.Assign (dev_r_ap.Get(1));
  Ipv4InterfaceContainer ifce_r_csma = ipv4h.Assign (dev_r_ap.Get(0));





  
  // ****************************************************************************************************************
  //                  Configure FDNetDevices and connect to TAP file descriptor 
  // ****************************************************************************************************************
  
  // Create an fd device, set a MAC address and point the device to the Linux device name.
  FdNetDeviceHelper fdNet;
  NetDeviceContainer tapDevs = fdNet.Install (nodes);
  NetDeviceContainer tapDevs1 = fdNet.Install (nodes);

  // lte path 
  Ptr<NetDevice> l0 = tapDevs.Get (0);
  Ptr<FdNetDevice> leftFdDev = l0->GetObject<FdNetDevice> ();
  leftFdDev->SetFileDescriptor (tun_open(tap_l));
  Ptr<NetDevice> r0= tapDevs.Get (1);
  Ptr<FdNetDevice> rightFdDev = r0->GetObject<FdNetDevice> ();
  rightFdDev->SetFileDescriptor (tun_open(tap_r));
  
  // wifi path 
  Ptr<NetDevice> l1 = tapDevs1.Get (0);
  Ptr<FdNetDevice> leftFdDev1 = l1->GetObject<FdNetDevice> ();
  leftFdDev1->SetFileDescriptor (tun_open(tap_l1));
  Ptr<NetDevice> r1= tapDevs1.Get (1);
  Ptr<FdNetDevice> rightFdDev1 = r1->GetObject<FdNetDevice> ();
  rightFdDev1->SetFileDescriptor (tun_open(tap_r1));

  // we have a FdNetDevices connected to TAP devices through fd, 
  // now install it on nodes and set IP addresses 
  Ptr<Ipv4> ipv4 = nodes.Get(0)->GetObject<Ipv4> ();  // get Ipv4 object from node 
  uint32_t interface = ipv4->AddInterface (leftFdDev1);
  Ipv4InterfaceAddress address = Ipv4InterfaceAddress (Ipv4Address ("15.0.0.1"), Ipv4Mask("255.0.0.0"));
  std::cout<<"Adress 1: " << address << std::endl; 
  ipv4->AddAddress (interface, address);              // set ip address to ifce
  // ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);
  
  ipv4 = nodes.Get(1)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (rightFdDev1);
  address = Ipv4InterfaceAddress (Ipv4Address ("14.0.0.1"), Ipv4Mask("255.0.0.0"));
  std::cout<<"Adress 2: " << address << std::endl; 
  ipv4->AddAddress (interface, address);
  // ipv4->SetMetric (interface, 1);
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
  // routing on left node                 
  // ********************************
  ipv4 = nodes.Get(0)->GetObject<Ipv4> ();
  Ptr<Ipv4StaticRouting> leftStaticRouting = ipv4RoutingHelper.GetStaticRouting (ipv4);
  leftStaticRouting->SetDefaultRoute (Ipv4Address ("16.0.0.1"), ifce_l_wifi.Get(0).second);

  // ******************************** 
  // routing on right node                
  // ********************************
  ipv4 = nodes.Get(1)->GetObject<Ipv4> (); 
  Ptr<Ipv4StaticRouting> rightStaticRouting = ipv4RoutingHelper.GetStaticRouting (ipv4);
  rightStaticRouting->SetDefaultRoute (Ipv4Address ("17.0.0.1"), ifce_r_csma.Get(0).second);
  

  // ****************************************************************************************************************
  //                        Configure APPLICATIONS 
  // ****************************************************************************************************************
  //
  // Create the ping application.  This application knows how to send
  // ICMP echo requests.  Setting up the packet sink manually is a bit
  // of a hassle and since there is no law that says we cannot mix the
  // helper API with the low level API, let's just use the helper.
  //
  NS_LOG_INFO ("Create V4Ping Appliation");
  Ptr<V4Ping> app = CreateObject<V4Ping> ();
  app->SetAttribute ("Remote", Ipv4AddressValue (remoteIp));
  app->SetAttribute ("Verbose", BooleanValue (true));
  nodes.Get(0)->AddApplication (app);
  app->SetStartTime (Seconds (1.0));
  app->SetStopTime (Seconds (21.0));
  
  //
  // Give the application a name.  This makes life much easier when constructing
  // config paths.
  //
  Names::Add ("app", app);

  //
  // Hook a trace to print something when the response comes back.
  //
  Config::Connect ("/Names/app/Rtt", MakeCallback (&PingRtt));

  wifiPhy.EnablePcapAll ("mp-wifi-lte", true);


  //
  // Now, do the actual emulation.
  //
  NS_LOG_INFO ("Run Emulation.");
  Simulator::Stop (Seconds (65.0));
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
}

