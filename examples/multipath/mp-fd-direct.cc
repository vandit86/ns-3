/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
//

#include "ns3/abort.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/csma-module.h"
#include "ns3/mobility-module.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TAPPingExample");

static void
PingRtt (std::string context, Time rtt)
{
  NS_LOG_UNCOND ("Received Response with RTT = " << rtt);
}

int
main (int argc, char *argv[])
{
  //
  // Allow the user to override any of the defaults at run-time, via
  // command-line arguments
  //
  std::string remoteIp = "11.0.0.2"; 
  double simTime = 60 ;
  bool enablePcap = false;  
  CommandLine cmd (__FILE__);
  cmd.AddValue ("remote", "Remote IP address (dotted decimal only please)", remoteIp);
  cmd.AddValue ("simTime", "simtime value", simTime);
  cmd.AddValue ("enablePcap", "If 'true' enable pcap generation on interfaces ", enablePcap);
  cmd.Parse (argc, argv);

  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  //
  // Create two ghost nodes.  The first will represent the virtual machine host
  // on the left side of the network; and the second will represent the VM on 
  // the right side.
  //
  NodeContainer nodes;
  nodes.Create (2);
  
  InternetStackHelper inet; 
  inet.Install (nodes); 
  
  // set possition and mobility model 
  Ptr<ListPositionAllocator> posAll = CreateObject<ListPositionAllocator> ();
  posAll->Add (Vector (0.0, 0.0, 0.0));
  posAll->Add (Vector (5, 0, 0));
  MobilityHelper mobility;
  mobility.SetPositionAllocator (posAll);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes); 

  
  // CsmaHelper csma;
  // csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  // csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds(1)));
  // NetDeviceContainer rightDev = csma.Install (right_m);
  
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
  devices = fdNet.Install (nodes.Get (0));
  Ptr<NetDevice> rightFdDev = devices.Get (0);
  rightFdDev->SetAttribute ("Address", Mac48AddressValue (Mac48Address::Allocate ()));

  fdNet.SetDeviceName ("vethLeft1");
  devices = fdNet.Install (nodes.Get (1));
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

  // node 1 left
  ipv4 = nodes.Get (1)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (leftFdDev1);
  address = Ipv4InterfaceAddress (Ipv4Address ("15.0.0.1"), Ipv4Mask ("255.0.0.0"));
  ipv4->AddAddress (interface, address);
  ipv4->SetUp (interface);

  // node 0 left
  ipv4 = nodes.Get (0)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (leftFdDev);
  address = Ipv4InterfaceAddress (Ipv4Address ("11.0.0.1"), Ipv4Mask ("255.0.0.0"));
  ipv4->AddAddress (interface, address);
  ipv4->SetUp (interface);

  // node 1 right
  ipv4 = nodes.Get (1)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (rightFdDev1);
  address = Ipv4InterfaceAddress (Ipv4Address ("14.0.0.1"), Ipv4Mask ("255.0.0.0"));
  ipv4->AddAddress (interface, address);
  ipv4->SetUp (interface);

  // node 0 right
  ipv4 = nodes.Get (0)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (rightFdDev);
  address = Ipv4InterfaceAddress (Ipv4Address ("13.0.0.1"), Ipv4Mask ("255.0.0.0"));
  ipv4->AddAddress (interface, address);
  ipv4->SetUp (interface);


  //
  // Create the ping application.  This application knows how to send
  // ICMP echo requests.  Setting up the packet sink manually is a bit
  // of a hassle and since there is no law that says we cannot mix the
  // helper API with the low level API, let's just use the helper.
  //
  // NS_LOG_INFO ("Create V4Ping Appliation");
  // Ptr<V4Ping> app = CreateObject<V4Ping> ();
  // app->SetAttribute ("Remote", Ipv4AddressValue (remoteIp));
  // app->SetAttribute ("Verbose", BooleanValue (true) );
  // nodes.Get(0)->AddApplication (app);
  // app->SetStartTime (Seconds (1.0));
  // app->SetStopTime (Seconds (21.0));
  
  //
  // Give the application a name.  This makes life much easier when constructing
  // config paths.
  //
  // Names::Add ("app", app);

  // //
  // // Hook a trace to print something when the response comes back.
  // //
  // Config::Connect ("/Names/app/Rtt", MakeCallback (&PingRtt));

  // wifiPhy.EnablePcapAll ("mp-wifi-lte", true);
  if (enablePcap)
    fdNet.EnablePcapAll ("mp-fd-direct", true);

  //
  // Now, do the actual emulation.
  //
  NS_LOG_INFO ("Run Emulation.");
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
}

