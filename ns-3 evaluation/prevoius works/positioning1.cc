#include "ns3/wifi-standards.h"
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
// #include "ns3/packet-sink.h"
// #include "ns3/packet-sink-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h"
#include <ns3/buildings-helper.h>
#include <ns3/hybrid-buildings-propagation-loss-model.h>
#include "ns3/oh-buildings-propagation-loss-model.h"
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("AbstacleAwarePositining");
#define CHANNEL_NUMBER 50

// ==================================================================
//            WI-FI
// ==================================================================

double
determineWifiChannelFrequencyHz (enum WifiStandard wifiStandard, uint32_t wifiChannelNumber)
{
  if (wifiStandard == WIFI_STANDARD_80211b || wifiStandard == WIFI_STANDARD_80211g ||
      wifiStandard == WIFI_STANDARD_80211n)
    return ((2407 + 5 * wifiChannelNumber) * (1e6));
  else
    return ((5e3 + 5 * wifiChannelNumber) * (1e6));
}

int
main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nWifi = 1;
  bool tracing = true;
  // std::string phyRate = "HtMcs7";
  double simulationTime = 100; /* Simulation time in seconds. */

  CommandLine cmd (__FILE__);
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);

  cmd.Parse (argc, argv);

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

  //define Ap and Stas
  NodeContainer wifiApNode;
  wifiApNode.Create (nWifi);
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (2);

  // Wi-Fi standard and frequency
  const enum WifiStandard wifiStandard =
      WIFI_STANDARD_80211ac; // Wi-Fi standard (WIFI_PHY_STANDARD_80211ac)                                                  // Wi-Fi channel number (50 @ 5250 MHz)
  // double wifiFrequencyHz =
  //     determineWifiChannelFrequencyHz (wifiStandard, CHANNEL_NUMBER); // Absolute frequency (in Hz)

  // Wi-Fi channel
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

  // Ptr<HybridBuildingsPropagationLossModel> propagationLossModel =
  //     CreateObject<HybridBuildingsPropagationLossModel> ();
  // // cancel shadowing effect
  // propagationLossModel->SetAttribute ("ShadowSigmaOutdoor", DoubleValue (0.0));
  // propagationLossModel->SetAttribute ("ShadowSigmaIndoor", DoubleValue (0.0));
  // propagationLossModel->SetAttribute ("ShadowSigmaExtWalls", DoubleValue (0.0))
  // wifiChannel.AddPropagationLoss ("propagationLossModel");

  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5e9),
                                  "SystemLoss", DoubleValue (1), "MinLoss", DoubleValue (0));

  // wifiChannel.AddPropagationLoss ("ns3::HybridBuildingsPropagationLossModel", "Frequency",
  //                                 DoubleValue (wifiFrequencyHz), "RooftopLevel", DoubleValue (20),
  //                                 "Environment", StringValue ("OpenAreas"), "Los2NlosThr",
  //                                 DoubleValue (200), "CitySize", StringValue ("Large"),
  //                                 "ShadowSigmaOutdoor", DoubleValue (0.0));

  /* Setup Physical Layer */

  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.Set ("ChannelNumber", UintegerValue (CHANNEL_NUMBER));
  wifiPhy.SetErrorRateModel ("ns3::YansErrorRateModel");
  wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
  // wifiPhy.Set ("ShortGuardEnabled", BooleanValue (false));

  // Wi-Fi Setting
  WifiHelper wifiHelper;
  wifiHelper.SetStandard (wifiStandard);
  wifiHelper.SetRemoteStationManager ("ns3::IdealWifiManager");

  /* Configure AP */
  WifiMacHelper wifiMac;
  // Ssid ssid = Ssid ("ns-3-ssid");
  Ssid ssid = Ssid ("wifi-default");
  wifiMac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  NetDeviceContainer apDevice;
  apDevice = wifiHelper.Install (wifiPhy, wifiMac, wifiApNode);

  /* Configure STA */
  wifiMac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ActiveProbing",
                   BooleanValue (true));
  NetDeviceContainer staDevice;
  staDevice = wifiHelper.Install (wifiPhy, wifiMac, wifiStaNodes);

  // wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
  /*wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (phyRate),
                                      "ControlMode", StringValue ("HtMcs0"));*/

  // Abstacle ############################################################
  double x_min = -5.0;
  double x_max = 5.0;
  double y_min = -5.0;
  double y_max = 5.0;
  double z_min = 0.0;
  double z_max = 20.0;
  Ptr<Building> b = CreateObject<Building> ();
  b->SetBoundaries (Box (x_min, x_max, y_min, y_max, z_min, z_max));
  b->SetBuildingType (Building::Residential);
  b->SetExtWallsType (Building::ConcreteWithoutWindows);
  b->SetNFloors (3);
  b->SetNRoomsX (3);
  b->SetNRoomsY (2);

  // BuildingsHelper::Install (wifiApNode);
  // BuildingsHelper::Install (wifiStaNodes);
  // // BuildingsHelper::MakeMobilityModelConsistent ();

  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 50.0));
  positionAlloc->Add (Vector (0.0, 10.0, 0.1));
  positionAlloc->Add (Vector (0.0, -11.0, 0.1));
  mobility.SetPositionAllocator (positionAlloc);

  mobility.Install (wifiApNode.Get (0));
  mobility.Install (wifiStaNodes.Get (0));
  mobility.Install (wifiStaNodes.Get (1));

  BuildingsHelper::Install (wifiApNode.Get (0));
  BuildingsHelper::Install (wifiStaNodes.Get (0));
  BuildingsHelper::Install (wifiStaNodes.Get (1));

  // Ptr<MobilityBuildingInfo> buildingInfoEnb = CreateObject<MobilityBuildingInfo> ();

  // Ptr<ConstantPositionMobilityModel> mmAp = CreateObject<ConstantPositionMobilityModel> ();
  // mmAp->SetPosition (Vector (0.0, -0.2927, 24.1942));
  // wifiApNode.Get (0)->AggregateObject (mmAp);
  // mmAp->AggregateObject (buildingInfoEnb); // operation usually done by BuildingsHelper::Install
  // buildingInfoEnb->MakeConsistent (mmAp);

  // Ptr<ConstantPositionMobilityModel> mmSt0 = CreateObject<ConstantPositionMobilityModel> ();
  // mmAp->SetPosition (Vector (0.0, 10.0, 0.1));
  // wifiStaNodes.Get (0)->AggregateObject (mmSt0);
  // mmSt0->AggregateObject (buildingInfoEnb); // operation usually done by BuildingsHelper::Install
  // buildingInfoEnb->MakeConsistent (mmSt0);

  // Ptr<ConstantPositionMobilityModel> mmSt1 = CreateObject<ConstantPositionMobilityModel> ();
  // mmAp->SetPosition (Vector (0.0, -11.0, 0.1));
  // wifiStaNodes.Get (1)->AggregateObject (mmSt1);
  // mmSt1->AggregateObject (buildingInfoEnb); // operation usually done by BuildingsHelper::Install
  // buildingInfoEnb->MakeConsistent (mmSt1);

  // Ptr<MobilityBuildingInfo> buildingInfoEnb = CreateObject<MobilityBuildingInfo> ();

  // wifiStaNodes.Get (0)->AggregateObject (
  //     buildingInfoEnb); // operation usually done by BuildingsHelper::Install
  // buildingInfoEnb->MakeConsistent (wifiStaNodes.Get (0));
  // wifiStaNodes.Get (1)->AggregateObject (
  //     buildingInfoEnb); // operation usually done by BuildingsHelper::Install
  // buildingInfoEnb->MakeConsistent (wifiStaNodes.Get (1));

  InternetStackHelper stack;
  stack.Install (wifiApNode);
  stack.Install (wifiStaNodes);

  Ipv4AddressHelper address;
  Address serverAddress;

  address.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign (apDevice);
  serverAddress = Address (apInterface.GetAddress (0));
  Ipv4InterfaceContainer staInterface;
  staInterface = address.Assign (staDevice);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // server
  UdpEchoServerHelper echoServer (9);

  ApplicationContainer serverApps = echoServer.Install (wifiApNode);

  /* Start Applications */
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));

  //client apInterface.GetAddress(0)
  UdpEchoClientHelper echoClient (serverAddress, 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (10000));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = echoClient.Install (wifiStaNodes.Get (0));
  // ApplicationContainer clientApps = echoClient.Install (wifiStaNodes.Get (1));
  clientApps.Add (echoClient.Install (wifiStaNodes.Get (1)));

  clientApps.Start (Seconds (1.1));
  clientApps.Stop (Seconds (10.0));

  if (tracing == true)
    {
      wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
      // pointToPoint.EnablePcapAll ("p2pcap");
      wifiPhy.EnablePcap ("Accesspoint", apDevice);
      wifiPhy.EnablePcap ("station", staDevice);
      // wifiPhy.EnablePcap ("station", staDevice1);
    }
  Simulator::Stop (Seconds (simulationTime + 1));
  AnimationInterface anim{"third2.xml"};

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  Simulator::Run ();

  monitor->SerializeToXmlFile ("thirdflow2.xml", true, true);

  Simulator::Destroy ();

  return 0;
}
