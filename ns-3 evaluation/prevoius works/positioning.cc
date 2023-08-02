
#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/buildings-module.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-module.h"
#include <ns3/buildings-module.h>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ThirdScriptExample");

int
main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nWifi = 1;
  bool tracing = true;
  std::string phyRate = "HtMcs7";
  double simulationTime = 100; /* Simulation time in seconds. */

  // uint32_t SentPackets = 0;
  // uint32_t ReceivedPackets = 0;
  // uint32_t LostPackets = 0;

  CommandLine cmd (__FILE__);
  //cmd.AddValue ("nCsma", "Number of \"extra\" CSMA nodes/devices", nCsma);
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);

  cmd.Parse (argc, argv);

  // The underlying restriction of 18 is due to the grid position
  // allocator's configuration; the grid layout will exceed the
  // bounding box if more than 18 nodes are provided.
  if (nWifi > 18)
    {
      std::cout << "nWifi should be 18 or less; otherwise grid layout exceeds the bounding box"
                << std::endl;
      return 1;
    }

  if (verbose)
    {
      LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

  // NodeContainer p2pNodes;
  // p2pNodes.Create (2);

  // PointToPointHelper pointToPoint;
  // pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  // pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  // NetDeviceContainer p2pDevices;
  // p2pDevices = pointToPoint.Install (p2pNodes);

  //define STA
  NodeContainer wifiStaNode0;
  wifiStaNode0.Create (1);
  NodeContainer wifiStaNode1;
  wifiStaNode1.Create (1);
  NodeContainer wifiApNode;
  wifiApNode.Create (1);

  //define AP
  // NodeContainer wifiApNode = p2pNodes.Get (0);

  //Ptr<HybridBuildingsPropagationLossModel> propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel> ();

  YansWifiChannelHelper wifiChannel;
  //wifiChannel.SetPropagationLossModel(propagationLossModel);
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5e9),
                                  "SystemLoss", DoubleValue (1), "MinLoss", DoubleValue (0));

  WifiMacHelper wifiMac;
  WifiHelper wifiHelper;
  wifiHelper.SetStandard (WIFI_STANDARD_80211ac);

  /* Setup Physical Layer */
  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.SetErrorRateModel ("ns3::YansErrorRateModel");
  wifiHelper.SetRemoteStationManager ("ns3::IdealWifiManager");

  /*wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (phyRate),
                                      "ControlMode", StringValue ("HtMcs0"));*/

  /* Configure AP */
  Ssid ssid = Ssid ("ns-3-ssid");
  wifiMac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  NetDeviceContainer apDevices;
  apDevices = wifiHelper.Install (wifiPhy, wifiMac, wifiApNode);

  /* Configure STA */
  wifiMac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ActiveProbing",
                   BooleanValue (false));
  NetDeviceContainer staDevice0;
  staDevice0 = wifiHelper.Install (wifiPhy, wifiMac, wifiStaNode0);

  NetDeviceContainer staDevice1;
  staDevice1 = wifiHelper.Install (wifiPhy, wifiMac, wifiStaNode1);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  // positionAlloc->Add (Vector (1000.0, 50.0, 0.0));
  positionAlloc->Add (Vector (5.0, -0.5, 47.0));
  positionAlloc->Add (Vector (0.0, 10.0, 0.0));
  positionAlloc->Add (Vector (0.0, -9.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  // mobility.Install (p2pNodes.Get (1));
  mobility.Install (wifiApNode);
  mobility.Install (wifiStaNode0);
  mobility.Install (wifiStaNode1);

  InternetStackHelper stack;
  stack.Install (wifiApNode);
  stack.Install (wifiStaNode0);
  stack.Install (wifiStaNode1);

  Ipv4AddressHelper address;

  // address.SetBase ("10.1.1.0", "255.255.255.0");
  // Ipv4InterfaceContainer p2pInterfaces;
  // p2pInterfaces = address.Assign (p2pDevices);

  address.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign (apDevices);
  // serverAddress = Address (apInterface.GetAddress);
  Ipv4InterfaceContainer staInterface0;
  staInterface0 = address.Assign (staDevice0);

  Ipv4AddressHelper address1;

  // address1.SetBase ("10.1.1.0", "255.255.255.0");
  // Ipv4InterfaceContainer apInterface;
  // apInterface = address1.Assign (apDevices);
  // serverAddress = Address (apInterface.GetAddress);
  Ipv4InterfaceContainer staInterface1;
  staInterface1 = address1.Assign (staDevice1);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // server
  UdpEchoServerHelper echoServer (9);

  ApplicationContainer serverApps = echoServer.Install (wifiApNode);

  /* Start Applications */
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));

  //client
  UdpEchoClientHelper echoClient (apInterface.GetAddress (0), 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (10000));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (4096));

  ApplicationContainer clientApps = echoClient.Install (wifiStaNode0);
  // ApplicationContainer clientApps = echoClient.Install (wifiStaNodes.Get (1));
  // clientApps.Add (echoClient.Install (wifiStaNodes.Get (1)));

  clientApps.Start (Seconds (1.1));
  clientApps.Stop (Seconds (10.0));

  // UdpEchoClientHelper echoClient1 (apInterface.GetAddress (0), 9);
  // echoClient1.SetAttribute ("MaxPackets", UintegerValue (1000));
  // echoClient1.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
  // echoClient1.SetAttribute ("PacketSize", UintegerValue (4096));

  // ApplicationContainer clientApps1 = echoClient1.Install (wifiStaNode1);
  // // ApplicationContainer clientApps = echoClient.Install (wifiStaNodes.Get (1));

  // clientApps1.Start (Seconds (1.2));
  // clientApps.Stop (Seconds (10.0));

  if (tracing == true)
    {
      wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
      // pointToPoint.EnablePcapAll ("p2pcap");
      wifiPhy.EnablePcap ("Accesspoint", apDevices);
      wifiPhy.EnablePcap ("station", staDevice0);
      wifiPhy.EnablePcap ("station", staDevice1);
    }
  Simulator::Stop (Seconds (simulationTime + 1));
  AnimationInterface anim{"third2.xml"};
  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}
