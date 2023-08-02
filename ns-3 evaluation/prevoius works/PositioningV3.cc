#include "ns3/wifi-standards.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
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
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/hybrid-buildings-propagation-loss-model.h>
#include "ns3/oh-buildings-propagation-loss-model.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/test.h"
#include "ns3/buildings-channel-condition-model.h"
#include "ns3/constant-position-mobility-model.h"
#include <ns3/vector.h>

#define APPS_SERVER_START_TIME 1
#define simulationTime 10
#define APPS_SERVER_STOP_TIME 10
#define CHANNEL_NUMBER 50
#define PacketSize 1024

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("AbstacleAwarePositining");

// ==================================================================
//            WI-FI
// ==================================================================

Ptr<PacketSink> sink;
uint64_t lastTotalRx = 0; /* The value of the last total received bytes */

double
determineWifiChannelFrequencyHz (enum WifiStandard wifiStandard, uint32_t wifiChannelNumber)
{
  if (wifiStandard == WIFI_STANDARD_80211b || wifiStandard == WIFI_STANDARD_80211g ||
      wifiStandard == WIFI_STANDARD_80211n)
    return ((2407 + 5 * wifiChannelNumber) * (1e6));
  else
    return ((5e3 + 5 * wifiChannelNumber) * (1e6));
}

void
CalculateThroughput ()
{
  Time now = Simulator::Now (); /* Return the simulator's virtual time. */
  double cur = (sink->GetTotalRx () - lastTotalRx) * (double) 8 /
               1e5; /* Convert Application RX Packets to MBits. */
  std::cout << (sink->GetTotalRx () - lastTotalRx) << "\t" << now.GetSeconds () << "s: \t" << cur
            << " Mbit/s" << std::endl;
  lastTotalRx = sink->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput);
}
int
main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nWifi = 1;
  bool tracing = true;
  std::string dataRate = "100Mbps";

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
  double wifiFrequencyHz =
      determineWifiChannelFrequencyHz (wifiStandard, CHANNEL_NUMBER); // Absolute frequency (in Hz)

  //   // Wi-Fi channel
  //   YansWifiChannelHelper wifiChannel;
  //   wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

  Ptr<HybridBuildingsPropagationLossModel> propagationLossModel =
      CreateObject<HybridBuildingsPropagationLossModel> ();

  // channel set Attribute
  propagationLossModel->SetAttribute ("Frequency", DoubleValue (wifiFrequencyHz));
  propagationLossModel->SetAttribute ("RooftopLevel", DoubleValue (20));
  propagationLossModel->SetAttribute ("Environment", StringValue ("SubUrban"));
  propagationLossModel->SetAttribute ("Los2NlosThr", DoubleValue (200));
  //   propagationLossModel->SetAttribute ("m_itu1411NlosThreshold", DoubleValue (200));
  propagationLossModel->SetAttribute ("CitySize", StringValue ("Small"));
  //   propagationLossModel->SetAttribute ("MinDistance", DoubleValue (0.5));

  //BuildingPropagationLossModel Attributes.
  propagationLossModel->SetAttribute ("ShadowSigmaOutdoor", DoubleValue (7.0));
  propagationLossModel->SetAttribute ("ShadowSigmaIndoor", DoubleValue (7.0));
  propagationLossModel->SetAttribute ("ShadowSigmaExtWalls", DoubleValue (5.0));
  propagationLossModel->SetAttribute ("InternalWallLoss", DoubleValue (5.0));

  Ptr<ConstantSpeedPropagationDelayModel> DelayLossModel =
      CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<YansWifiChannel> wifiChannel = CreateObject<YansWifiChannel> ();
  wifiChannel->SetPropagationLossModel (propagationLossModel);
  wifiChannel->SetPropagationDelayModel (DelayLossModel);

  /* Setup Physical Layer */

  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel (wifiChannel);
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

  // MobilityHelper mobility;
  // mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  // positionAlloc->Add (Vector (0.0, -10.0, 17.0));
  // positionAlloc->Add (Vector (0.0, 10.0, 0.1));
  // positionAlloc->Add (Vector (0.0, -11.0, 0.1));
  // mobility.SetPositionAllocator (positionAlloc);

  // ns3::Vector positionap = Vector (0.0, -10.0, 17.0);
  // ns3::Vector positionue1 = Vector (0.0, 10.0, 0.1);
  // ns3::Vector positionue1 = Vector (0.0, -11.0, 0.1);

  Ptr<BuildingsChannelConditionModel> condModel = CreateObject<BuildingsChannelConditionModel> ();
  Ptr<MobilityModel> ap = CreateObject<ConstantPositionMobilityModel> ();
  wifiApNode.Get (0)->AggregateObject (ap);
  Ptr<MobilityModel> ue1 = CreateObject<ConstantPositionMobilityModel> ();
  wifiStaNodes.Get (0)->AggregateObject (ue1);
  Ptr<MobilityModel> ue2 = CreateObject<ConstantPositionMobilityModel> ();
  wifiStaNodes.Get (1)->AggregateObject (ue2);

  // mobility.Install (wifiApNode.Get (0));
  // mobility.Install (wifiStaNodes.Get (0));
  // mobility.Install (wifiStaNodes.Get (1));

  BuildingsHelper::Install (wifiApNode.Get (0));
  BuildingsHelper::Install (wifiStaNodes.Get (0));
  BuildingsHelper::Install (wifiStaNodes.Get (1));

  ap->SetPosition (Vector (0.0, -3.0, 40.0));
  ue1->SetPosition (Vector (0.0, 10.0, 0.1));
  ue2->SetPosition (Vector (0.0, -11.0, 0.1));
  Ptr<MobilityBuildingInfo> buildingInfoAp = ap->GetObject<MobilityBuildingInfo> ();
  buildingInfoAp->MakeConsistent (ap);
  Ptr<MobilityBuildingInfo> buildingInfoUe1 = ue1->GetObject<MobilityBuildingInfo> ();
  buildingInfoUe1->MakeConsistent (ue1);
  Ptr<ChannelCondition> cond1 = condModel->GetChannelCondition (ap, ue1);
  Ptr<ChannelCondition> cond2 = condModel->GetChannelCondition (ap, ue2);
  std::cout << "cannel between Ap and Ue1 is: " << cond1->GetLosCondition () << std::endl;
  std::cout << "cannel between Ap and Ue2 is: " << cond2->GetLosCondition () << std::endl;

  InternetStackHelper stack;
  stack.Install (wifiApNode);
  stack.Install (wifiStaNodes);

  Ipv4AddressHelper address;

  address.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign (apDevice);
  Ipv4InterfaceContainer staInterface;
  staInterface = address.Assign (staDevice);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // Install UDP sink on receiser

  PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory",
                               InetSocketAddress (Ipv4Address::GetAny (), 9));
  sinkHelper.SetAttribute ("StartTime", TimeValue (Seconds (APPS_SERVER_START_TIME)));
  // sinkHelper.SetAttribute ("StopTime", TimeValue (Seconds (APPS_SERVER_STOP_TIME)));

  ApplicationContainer sinkApp = sinkHelper.Install (wifiApNode);
  sink = StaticCast<PacketSink> (sinkApp.Get (0));

  // Install UDP Transmitter on the stations
  OnOffHelper client ("ns3::UdpSocketFactory", (InetSocketAddress (apInterface.GetAddress (0), 9)));
  client.SetAttribute ("PacketSize", UintegerValue (PacketSize));
  client.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  client.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  client.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
  client.SetAttribute ("StartTime", TimeValue (Seconds (APPS_SERVER_START_TIME)));
  // client.SetAttribute ("StopTime", TimeValue (Seconds (simulationTime)));
  ApplicationContainer staApp = client.Install (wifiStaNodes.Get (0));
  staApp.Add (client.Install (wifiStaNodes.Get (1)));

  //   staApp.Start (Seconds (1.1));
  //   staApp.Stop (Seconds (10.0));

  if (tracing == true)
    {
      wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
      wifiPhy.EnablePcap ("Accesspoint", apDevice);
      wifiPhy.EnablePcap ("station", staDevice);
    }
  Simulator::Schedule (Seconds (1.1), &CalculateThroughput);

  Simulator::Stop (Seconds (simulationTime + 1));
  AnimationInterface anim{"positioning.xml"};
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  Simulator::Run ();
  monitor->SerializeToXmlFile ("positioningflow.xml", true, true);
  double averageThroughput = ((sink->GetTotalRx () * 8) / (1e6 * simulationTime));

  Simulator::Destroy ();

  if (averageThroughput < 50)
    {
      NS_LOG_ERROR ("Obtained throughput is not in the expected boundaries!");
      exit (1);
    }
  std::cout << "\nAverage throughput: " << averageThroughput << " Mbit/s" << std::endl;
  return 0;
}
