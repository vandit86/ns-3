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

//
// This is an illustration of how one could use virtualization techniques to
// allow running applications on virtual machines talking over simulated
// networks.
//
// The actual steps required to configure the virtual machines can be rather
// involved, so we don't go into that here.  Please have a look at one of
// our HOWTOs on the nsnam wiki for more details about how to get the 
// system confgured.  For an example, have a look at "HOWTO Use Linux 
// Containers to set up virtual networks" which uses this code as an 
// example.
//
// The configuration you are after is explained in great detail in the 
// HOWTO, but looks like the following:
//
//  +----------+                           +----------+
//  | virtual  |                           | virtual  |
//  |  Linux   |                           |  Linux   |
//  |   Host   |                           |   Host   |
//  |          |                           |          |
//  |   eth0   |                           |   eth0   |
//  +----------+                           +----------+
//       |                                      |
//  +----------+                           +----------+
//  |  Linux   |                           |  Linux   |
//  |  Bridge  |                           |  Bridge  |
//  +----------+                           +----------+
//       |                                      |
//  +------------+                       +-------------+
//  | "tap-left" |                       | "tap-right" |
//  +------------+                       +-------------+
//       |           n0            n1           |
//       |       +--------+    +--------+       |
//       +-------|  tap   |    |  tap   |-------+
//               | bridge |    | bridge |
//               +--------+    +--------+
//               |  CSMA  |    |  CSMA  |
//               +--------+    +--------+
//                   |             |
//                   |             |
//                   |             |
//                   ===============
//                      CSMA LAN
//
#include <iostream>
#include <fstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/csma-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/config-store-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("MPTapCsmaVirtualMachineExample");

void ThroughputMonitor (FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon)
{
  flowMon->CheckForLostPackets();
  std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
  Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
  {
    Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
    std::cout<<"Flow ID                 : " << stats->first <<" ; "<< fiveTuple.sourceAddress <<" -----> "<<fiveTuple.destinationAddress<<std::endl;
//  std::cout<<"Tx Packets = " << stats->second.txPackets<<std::endl;
//  std::cout<<"Rx Packets = " << stats->second.rxPackets<<std::endl;
    std::cout<<"Duration                : "<<stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds()<<std::endl;
    std::cout<<"Last Received Packet    : "<< stats->second.timeLastRxPacket.GetSeconds()<<" Seconds"<<std::endl;
    std::cout<<"Throughput: " << stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024/1024  << " Mbps"<<std::endl;
    std::cout<<"---------------------------------------------------------------------------"<<std::endl;
  }

 // Rescheduling the next call to this function to print throughput
 Simulator::Schedule(Seconds(3),&ThroughputMonitor, fmhelper, flowMon);

}

int 
main (int argc, char *argv[])
{
  std::cout << "Start simulation "<< std::endl;
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);

  //Config::SetDefault ("ns3::DropTailQueue<Packet>::MaxSize", StringValue ("100p"));
  
  
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
  NodeContainer nodes;
  nodes.Create (2);

  NodeContainer middle;
  middle.Create(1);     // create one node in the middle 

  NodeContainer left_m (nodes.Get(0), middle.Get(0));
  NodeContainer right_m (nodes.Get(1), middle.Get(0));

  // Use a CsmaHelper to get a CSMA channel created, and the needed net 
  // devices installed on both of the nodes.  The data rate and delay for the
  // channel can be set through the command-line parser.  For example,
  //
  // ./waf --run "tap=csma-virtual-machine --ns3::CsmaChannel::DataRate=10000000"
  //
  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (5)));
  NetDeviceContainer leftDev = csma.Install(left_m); 
  
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (5)));
  NetDeviceContainer rightDev = csma.Install (right_m);
  
  //internet for middle 
  InternetStackHelper inet; 
  inet.Install (middle);

  // Assign adress to AP wifi inface
  Ipv4AddressHelper ipv4h; 
  ipv4h.SetBase ("15.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer middle1iface = ipv4h.Assign (leftDev.Get(1));
  ipv4h.SetBase ("14.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer middle2iface = ipv4h.Assign (rightDev.Get(1));

  //
  // Use the TapBridgeHelper to connect to the pre-configured tap devices for 
  // the left side.  We go with "UseBridge" mode since the CSMA devices support
  // promiscuous mode and can therefore make it appear that the bridge is 
  // extended into ns-3.  The install method essentially bridges the specified
  // tap to the specified CSMA device.
  //
  TapBridgeHelper tapBridge; 
  tapBridge.SetAttribute ("Mode", StringValue ("UseLocal"));
   
  // tapBridge.SetAttribute ("DeviceName", StringValue ("tap-left"));
  // tapBridge.Install (nodes.Get (0), leftDev.Get (0));
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-left-1"));
  tapBridge.Install (nodes.Get (0), leftDev.Get (0));
  
  //
  // Connect the right side tap to the right side CSMA device on the right-side
  // ghost node.
  //
  // tapBridge.SetAttribute ("DeviceName", StringValue ("tap-right"));
  // tapBridge.Install (nodes.Get (1), devices.Get (1));
  
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-right-1"));
  tapBridge.Install (nodes.Get (1), rightDev.Get (0));

  csma.EnablePcapAll("mp-csma-3",false);
  //csma.EnablePcap() 
  //
  // Flow monitor
  // Ptr<FlowMonitor> flowMonitor;
  // FlowMonitorHelper flowHelper;
  // flowMonitor = flowHelper.InstallAll(); 

  // scheduling throughput to be printed every 3 seconds
  //ThroughputMonitor(&flowHelper, flowMonitor);


  // Run the simulation for ten minutes to give the user time to play around
  //

  //ns3::Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
  
  // Create the animation object and configure for specified output
  //AnimationInterface anim ("mp-anim.xml");
  //anim.EnablePacketMetadata (); // Optional

  // Output config store to txt format
  // Config::SetDefault ("ns3::TcpL4Protocol::SocketType", StringValue ("ns3::TcpNewReno"));
  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
  ConfigStore outputConfig2;
  outputConfig2.ConfigureDefaults ();
  outputConfig2.ConfigureAttributes ();
   

  std::cout << "Stop simulation \n"; 
  Simulator::Stop (Seconds (600.0));
  Simulator::Run ();
  Simulator::Destroy ();
  
  //flowMonitor->SerializeToXmlFile("mp-monitor.xml", true, true);
}
