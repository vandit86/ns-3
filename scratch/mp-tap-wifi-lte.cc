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
 */

#include <iostream>
#include <fstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"

#include "ns3/csma-module.h"
#include "ns3/internet-module.h"

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include <ns3/buildings-helper.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TapWifiVirtualMachineExample");

int
main (int argc, char *argv[])
{
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);

  // defaults for LTE 
  // ConfigStore inputConfig;
  // inputConfig.ConfigureDefaults ();
  // bool useCa = false;

  std::string phyMode ("OfdmRate6MbpsBW10MHz"); // for wave configuration
  
  //
  // We are interacting with the outside, real, world.  This means we have to
  // interact in real-time and therefore means we have to use the real-time
  // simulator and take the time to calculate checksums.
  //
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  //
  // Create two ghost nodes.  The first will represent the virtual machine host
  // on the left side of the network; and the second will represent the VM on
  // the right side.
  //
  NodeContainer left;
  left.Create (1);

  NodeContainer right; 
  right.Create (1); 
  
  //
  // We need location information since we are talking about wifi, so add a
  // constant position to the ghost nodes.
  //
  MobilityHelper mobility;
  // Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  // positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  // positionAlloc->Add (Vector (5.0, 0.0, 0.0));
  // mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(left); 
  //BuildingsHelper::Install (left);

  //mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(right); 
  //BuildingsHelper::Install (right);

  
  // ***************************
  // configure LTE devices 
  // ***************************
  std::cout << "Create LTE dev..";
  
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();  // helper 
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  
  NetDeviceContainer devices_lte_eNb = lteHelper->InstallEnbDevice (right); 
  NetDeviceContainer devices_lte = lteHelper->InstallUeDevice (left);           // 0
  // devices_lte.Add(lteHelper->InstallEnbDevice (right));                         // 1


  // Install the IP stack on the UEs
  InternetStackHelper internet;
  internet.Install (left);

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (devices_lte));
  // Assign IP address to UEs
  Ipv4StaticRoutingHelper ipv4RoutingHelper;

  Ptr<Node> ueNode = left.Get (0);
  // Set the default gateway for the UE
  Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
  ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

  std::cout << "OK" << std::endl; 

  std::cout << "Attach LTE dev.."; 
  // Attach a UE to a eNB
  // lteHelper->Attach (devices_lte.Get(0), devices_lte.Get (1));
  lteHelper->Attach (devices_lte, devices_lte_eNb.Get (0));
  std::cout << "OK" << std::endl; 

  // // Activate a data radio bearer
  // enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
  // EpsBearer bearer (q);
  // lteHelper->ActivateDataRadioBearer (devices_lte.Get(0), bearer);
  // // lteHelper->EnableTraces ();
  // std::cout << "Activate radio bearer LTE dev OK  " << std::endl; 
  // ***************************

   // ***************************
  // WiFI adhoc
  // We're going to use 802.11 A so set up a wifi helper to reflect that.
  // ***************************
  //
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
                                StringValue ("OfdmRate54Mbps"));
  // No reason for pesky access points, so we'll use an ad-hoc network.
  //
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");

  //
  // Configure the physical layer.
  //
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel (wifiChannel.Create ());
  //
  // Install the wireless devices onto our ghost nodes.
  //
  NetDeviceContainer devices_wifi = wifi.Install (wifiPhy, wifiMac, left.Get(0)); // create wifi dev container, atrting with left  
  devices_wifi.Add(wifi.Install (wifiPhy, wifiMac, right.Get(0)));                // add wifi device from right 


  // ***************************
  // CSMA to bridge
  // ***************************
  CsmaHelper csma;
  //csma.Install()
  // csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  // csma.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (5)));
  NetDeviceContainer devices_csma_left = csma.Install (left.Get(0));
  NetDeviceContainer devices_csma_right = csma.Install (right.Get(0));


  // ***************************
  // TAP devices 
  // ***************************
  std::cout << "Attach TAP devices.. "; 
  //
  // Use the TapBridgeHelper to connect to the pre-configured tap devices for
  // the left side.  We go with "UseLocal" mode since the wifi devices do not
  // support promiscuous mode (because of their natures0.  This is a special
  // case mode that allows us to extend a linux bridge into ns-3 IFF we will
  // only see traffic from one other device on that bridge.  That is the case
  // for this configuration.
  //
  TapBridgeHelper tapBridge;
  tapBridge.SetAttribute ("Mode", StringValue ("UseLocal"));
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-left"));
  tapBridge.Install (left.Get (0), devices_wifi.Get (0));
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-left-1"));
  tapBridge.Install (left.Get (0), devices_csma_left.Get (0));

  //
  // Connect the right side tap to the right side wifi device on the right-side
  // ghost node.
  //
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-right"));
  tapBridge.Install (right.Get (0), devices_wifi.Get (1));
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-right-1"));
  tapBridge.Install (right.Get (0), devices_csma_right.Get (0));

  std::cout << "OK " << std::endl; 

  //ns3::Ipv4GlobalRoutingHelper::PopulateRoutingTables (); 

  // ENABLE PCAP
  //wifiPhy.EnablePcapAll ("mp-wifi-lte", true);
  //
  // Run the simulation for ten minutes to give the user time to play around
  std::cout << "Start simulation " << std::endl;
  //
  Simulator::Stop (Seconds (600.));
  Simulator::Run ();
  Simulator::Destroy ();
}
