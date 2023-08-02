#include "ns3/applications-module.h"
#include "ns3/buildings-channel-condition-model.h"
#include "ns3/buildings-module.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-monitor.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/oh-buildings-propagation-loss-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/snr-tag.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/timestamp-tag.h"
#include "ns3/wifi-phy-state-helper.h"
#include "ns3/wifi-standards.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/buildings-module.h>
#include <ns3/hybrid-buildings-propagation-loss-model.h>
#include <ns3/itu-r-1411-los-propagation-loss-model.h>
#include <ns3/itu-r-1411-nlos-over-rooftop-propagation-loss-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/tag.h>
#include <ns3/vector.h>

#include <fstream>

#define CHANNEL_BW 160
#define OnOff_START_TIME (SINK_START_TIME + 1)
#define OnOff_STOP_TIME 40
#define SINK_START_TIME 30
#define SINK_STOP_TIME (OnOff_STOP_TIME + 2)
#define Simulation_Start_Time (OnOff_START_TIME)
#define Simulation_Stop_Time (SINK_STOP_TIME - 3)

#define CHANNEL_NUMBER 50
#define PacketSize 1024
// Define time interval (in seconds) to collect samples for delay results
#define DELAY_SAMPLE_INTERVAL 0.01

// ==================================================================
// ==================================================================

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AbstacleAwarePositining");


Ptr<PacketSink> sink;
uint64_t lastTotalRx = 0; /* The value of the last total received bytes */
double txBytesCounter = 0;
Time delayCounter;
double rxByteCounter = 0;

// determineWifiChannelFrequencyHz=========================================================

double
determineWifiChannelFrequencyHz(enum WifiStandard wifiStandard, uint32_t wifiChannelNumber)
{
    if (wifiStandard == WIFI_STANDARD_80211b || wifiStandard == WIFI_STANDARD_80211g ||
        wifiStandard == WIFI_STANDARD_80211n)
        return ((2407 + 5 * wifiChannelNumber) * (1e6));
    else
        return ((5e3 + 5 * wifiChannelNumber) * (1e6));
}

// Set antenna TX gain
void
SetTxGain(int nodeID, double value)
{
    std::ostringstream oss;
    oss << "/NodeList/" << nodeID << "/DeviceList/*/$ns3::WifiNetDevice/Phy/TxGain";
    Config::Set(oss.str(), DoubleValue(value));
}

void
SetTxPower(int nodeID, double value)
{
    std::ostringstream oss;
    oss << "/NodeList/" << nodeID << "/DeviceList/*/$ns3::WifiNetDevice/Phy/TxPowerStart";
    Config::Set(oss.str(), DoubleValue(value));

    oss.str("");
    oss.clear();

    oss << "/NodeList/" << nodeID << "/DeviceList/*/$ns3::WifiNetDevice/Phy/TxPowerEnd";
    Config::Set(oss.str(), DoubleValue(value));
}

void
SendPacket(Ptr<const Packet> pkt)
{
    
    TimestampTag timestamp;

    // timestampTag.SetTimestamp(Simulator::Now());
    // pkt->AddByteTag(timestampTag);
    // // std::cout << "timestamptaged" << std::endl;
    // // std::cout << Simulator::Now ().GetSeconds () << "\t" << pkt->GetSize () << std::endl;
    txBytesCounter += pkt->GetSize();
}

void
CalculateThroughput()
{
    Time duration = MilliSeconds(1000);
    Time now = Simulator::Now(); /* Return the simulator's virtual time. */
    double cur = (sink->GetTotalRx() - lastTotalRx) * 8.0 /
                 (1e6 * duration.GetSeconds()); /* Convert Application RX Packets to MBits. */
    std::ofstream OFS("/home/kamran/ns-3/ns-allinone-3.38/ns-3.38/scratch/UAV/result",
                      std::ios_base::app);
    if (OFS.is_open())
    {
        OFS << (sink->GetTotalRx() - lastTotalRx) << "\t" << now.GetSeconds() << "s: \t" << cur
            << " Mbit/s" << std::endl;
        OFS.close();
    }
    std::cout << sink->GetTotalRx() << "\t" << now.GetSeconds() << "s: \t" << cur << " Mbit/s"
              << std::endl;
    lastTotalRx = sink->GetTotalRx();
    Simulator::Schedule(duration, &CalculateThroughput);
}

void
ReceivePacket(Ptr<const Packet> pktRec, const Address& addr)
{
     //   SnrTag snrTag;
    TimestampTag timestampTag;
    Time timeTx;
    /* Piece of code for the statistics of a single flow */
    // InetSocketAddress address = InetSocketAddress::ConvertFrom(addr);
    // if (address.GetIpv4() == "10.0.0.1")
    //{
    /*if (p->PeekPacketTag(snrTag))
          {
          std::cout << "SNR: " << snrTag.Get() << std::endl;
          } */

    if (pktRec->FindFirstMatchingByteTag(timestampTag))
    {
        std::cout << "I am here" << std::endl;
        //   double previousSimulationTime = 0;
        timeTx = timestampTag.GetTimestamp();
        delayCounter += (Simulator::Now() - timeTx);
        std::cout << "timestamp" << timeTx << std::endl;

        //   if (Simulator::Now ().GetSeconds () >= (previousSimulationTime +
        //   DELAY_SAMPLE_INTERVAL))
        //     {
        //       std::cout << Simulator::Now ().GetSeconds () << ";"
        //                 << Simulator::Now ().GetNanoSeconds () - timeTx.GetNanoSeconds () <<
        //                 std::endl;
        //       previousSimulationTime = Simulator::Now ().GetSeconds ();
        //     }
    }
    //   else
    //     {
    //       std::cout << "no timestamp" << std::endl;
    //     }
    rxByteCounter += pktRec->GetSize();
    //}
}

#include "coords.h"

int
main(int argc, char* argv[])
{
    CoordsList coordsListUe =
        loadCoordsFromFile("/home/kamran/ns-3/ns-allinone-3.38/ns-3.38/scratch/UAV/ue.txt");
    CoordsList coordsListAp{
        loadCoordsFromFile("/home/kamran/ns-3/ns-allinone-3.38/ns-3.38/scratch/UAV/AP.txt")};
    dumpCoords(std::cout, coordsListUe, "UE:");
    dumpCoords(std::cout, coordsListAp, "AP:");
    bool verbose = true;
    uint32_t nWifi = static_cast<uint32_t>(coordsListAp.size()),
             nSta{static_cast<uint32_t>(coordsListUe.size())};

    bool tracing = true;
    // std::string dataRate = "10Mbps";

    CommandLine cmd(__FILE__);
    cmd.AddValue("nWifi", "Number of wifi STA devices", nWifi);
    cmd.AddValue("verbose", "Tell echo applications to log if true", verbose);
    cmd.AddValue("tracing", "Enable pcap tracing", tracing);

    cmd.Parse(argc, argv);

    if (nWifi > 18)
    {
        std::cout << "nWifi should be 18 or less; otherwise grid layout exceeds the bounding box"
                  << std::endl;
        return 1;
    }

    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

    // define Ap and Stas
    NodeContainer wifiApNode;
    wifiApNode.Create(nWifi);
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nSta);

    // Wi-Fi standard and frequency
    const enum WifiStandard wifiStandard =
        WIFI_STANDARD_80211ac; // Wi-Fi standard (WIFI_PHY_STANDARD_80211ac) // Wi-Fi channel number
                               // (50 @ 5250 MHz)
    double wifiFrequencyHz =
        determineWifiChannelFrequencyHz(wifiStandard, CHANNEL_NUMBER); // Absolute frequency (in Hz)

    // Buildings
    double x_min = -5.0;
    double x_max = 5.0;
    double y_min = -5.0;
    double y_max = 5.0;
    double z_min = 0.0;
    double z_max = 20.0;
    Ptr<Building> b = CreateObject<Building>();
    b->SetBoundaries(Box(x_min, x_max, y_min, y_max, z_min, z_max));
    b->SetBuildingType(Building::Residential);
    b->SetExtWallsType(Building::ConcreteWithoutWindows);
    b->SetNFloors(3);
    b->SetNRoomsX(3);
    b->SetNRoomsY(2);

    Ptr<MobilityModel> ap{};
    Ptr<MobilityBuildingInfo> buildingInfoAp = CreateObject<MobilityBuildingInfo>();
    for (uint32_t i = 0; i < nWifi; ++i)
    {
        ap = CreateObject<ConstantPositionMobilityModel>();
        ap->SetPosition(Vector(coordsListAp[i].x, coordsListAp[i].y, coordsListAp[i].z));
        wifiApNode.Get(i)->AggregateObject(ap);

        BuildingsHelper::Install(wifiApNode.Get(i));

        buildingInfoAp = ap->GetObject<MobilityBuildingInfo>();
        buildingInfoAp->MakeConsistent(ap);
    }
    Ptr<MobilityModel> ue{};
    Ptr<MobilityBuildingInfo> buildingInfoUe = CreateObject<MobilityBuildingInfo>();

    for (uint32_t i{}; i < nSta; ++i)
    {
        ue = CreateObject<ConstantPositionMobilityModel>();
        ue->SetPosition(Vector(coordsListUe[i].x, coordsListUe[i].y, coordsListUe[i].z));

        wifiStaNodes.Get(i)->AggregateObject(ue);
        BuildingsHelper::Install(wifiStaNodes.Get(i));

        buildingInfoUe = ue->GetObject<MobilityBuildingInfo>();
        buildingInfoUe->MakeConsistent(ue);
    }

    Ptr<BuildingsChannelConditionModel> condModel = CreateObject<BuildingsChannelConditionModel>();

    std::vector<std::string> LoSList;
    Ptr<ChannelCondition> cond{};
    uint32_t nLos{};
    ap = wifiApNode.Get(0)->GetObject<ConstantPositionMobilityModel>(
        ConstantPositionMobilityModel::GetTypeId());

    for (uint32_t i{}; i < nSta; ++i)
    {
        ue = wifiStaNodes.Get(i)->GetObject<ConstantPositionMobilityModel>(
            ConstantPositionMobilityModel::GetTypeId());
        cond = condModel->GetChannelCondition(ap, ue);
        std::cout << "Channel between the AP and the UE #" << i
                  << " is: " << cond->GetLosCondition() << std::endl;
        LoSList.push_back(std::to_string(!cond->GetLosCondition()));

        if (cond->GetLosCondition() == ChannelCondition::LosConditionValue::LOS)
        {
            ++nLos;
        }
    }
    std::vector<double> LoSLoss;
    Ptr<PropagationLossModel> lossModel1{};
    Ptr<ItuR1411LosPropagationLossModel> lModelLoS =
        CreateObject<ItuR1411LosPropagationLossModel>();
    lossModel1 = lModelLoS;
    lossModel1->SetAttribute("Frequency", DoubleValue(wifiFrequencyHz));
    for (uint32_t i{}; i < nSta; ++i)
    {
        ue = wifiStaNodes.Get(i)->GetObject<ConstantPositionMobilityModel>(
            ConstantPositionMobilityModel::GetTypeId());
        double loss = lModelLoS->GetLoss(ap, ue);
        LoSLoss.push_back(loss);
        std::cout << "In LoS Condition Loss(AP, UE#" << i << "):" << loss << std::endl;
    }

    std::vector<double> NLoSLoss;
    Ptr<PropagationLossModel> lossModel2{};
    Ptr<ItuR1411NlosOverRooftopPropagationLossModel> lModelNLoS =
        CreateObject<ItuR1411NlosOverRooftopPropagationLossModel>();
    lossModel2 = lModelNLoS;
    lossModel2->SetAttribute("Frequency", DoubleValue(wifiFrequencyHz));
    lossModel2->SetAttribute("RooftopLevel", DoubleValue(20));
    lossModel2->SetAttribute("Environment", StringValue("SubUrban"));
    lossModel2->SetAttribute("StreetsOrientation", DoubleValue(45));
    lossModel2->SetAttribute("CitySize", StringValue("Small"));

    lossModel2->SetAttribute("BuildingSeparation", DoubleValue(100));
    lossModel2->SetAttribute("BuildingsExtend", DoubleValue(10));
    lossModel2->SetAttribute("StreetsWidth", DoubleValue(200));

    for (uint32_t i{}; i < nSta; ++i)
    {
        ue = wifiStaNodes.Get(i)->GetObject<ConstantPositionMobilityModel>(
            ConstantPositionMobilityModel::GetTypeId());
        double loss = lModelNLoS->GetLoss(ap, ue);

        NLoSLoss.push_back(loss);
        std::cout << "In NLoS Conditions Loss(AP, UE#" << i << "): " << loss << std::endl;
    }

    Ptr<ConstantSpeedPropagationDelayModel> DelayLossModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();

    Ptr<YansWifiChannel> wifiChannel = CreateObject<YansWifiChannel>();
    wifiChannel->SetPropagationDelayModel(DelayLossModel);

    //  Apply Propagation Loss Models
    if (nLos) // All have LOS
    {
        wifiChannel->SetPropagationLossModel(lossModel1);
        std::clog << "Propagation Loss Model is: ItuR1411LosPropagationLossModel" << std::endl;
    }
    else
    {
        wifiChannel->SetPropagationLossModel(lossModel2);
        std::clog << "Propagation Loss Model is: ItuR1411NlosOverRooftopPropagationLossModel"
                  << std::endl;

        // channel set Attribute
    }

    /* Setup Physical Layer */

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel);
    wifiPhy.Set("ChannelNumber", UintegerValue(CHANNEL_NUMBER));
    wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel");
    wifiPhy.Set("TxPowerLevels", UintegerValue(1));

    // wifiPhy.Set ("ShortGuardEnabled", BooleanValue (false));
    // Set Tx Power
    // wifiPhy.Set("TxPowerStart", DoubleValue(16.0));
    // wifiPhy.Set("TxPowerEnd", DoubleValue(16.0));

    // Wi-Fi Setting
    WifiHelper wifiHelper;
    wifiHelper.SetStandard(wifiStandard);
    wifiHelper.SetRemoteStationManager("ns3::IdealWifiManager");

    /* Configure AP */
    WifiMacHelper wifiMac;
    // Ssid ssid = Ssid ("ns-3-ssid");
    Ssid ssid = Ssid("wifi-default");
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevice;
    apDevice = wifiHelper.Install(wifiPhy, wifiMac, wifiApNode);

    /* Configure STA */
    wifiMac.SetType("ns3::StaWifiMac",
                    "Ssid",
                    SsidValue(ssid),
                    "ActiveProbing",
                    BooleanValue(true));
    NetDeviceContainer staDevice;
    staDevice = wifiHelper.Install(wifiPhy, wifiMac, wifiStaNodes);

    // set attenuation for NLoS Links
    bool allZero = true;
    for (const auto& elem : LoSList)
    {
        if (elem == "1")
        {
            allZero = false;
            break;
        }
    }

    for (size_t i = 0; i < LoSList.size(); ++i)
    {
        if (LoSList[i] == "0" && !allZero)

        {
            std::cout << "Calculated loss: " << 16.0206 - NLoSLoss[i] + LoSLoss[i] << std::endl;
            SetTxPower(wifiStaNodes.Get(i)->GetId(), (16.0206 - NLoSLoss[i] + LoSLoss[i]));
            std::cout << "NLOS with ue # :" << wifiStaNodes.Get(i)->GetId() << std::endl;
        }
    }

    // bandwith
    Config::Set("/NodeList/*/DeviceList/*/F/Phy/ChannelWidth", UintegerValue(CHANNEL_BW));
    Config::Set("/NodeList/*/DeviceList/*/F/Phy/GuardInterval", TimeValue(NanoSeconds(800)));

    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;

    address.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface;
    apInterface = address.Assign(apDevice);
    Ipv4InterfaceContainer staInterface;
    staInterface = address.Assign(staDevice);

    /* Populate routing table */
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Install UDP sink on receiser
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), 9));
    sinkHelper.SetAttribute("StartTime", TimeValue(Seconds(SINK_START_TIME)));
    sinkHelper.SetAttribute("StopTime", TimeValue(Seconds(SINK_STOP_TIME)));

    ApplicationContainer sinkApp = sinkHelper.Install(wifiApNode.Get(0));
    sink = StaticCast<PacketSink>(sinkApp.Get(0));

    

    // Install UDP Transmitter on the stations

    OnOffHelper client("ns3::UdpSocketFactory", InetSocketAddress(apInterface.GetAddress(0), 9));
    client.SetAttribute("StartTime", TimeValue(Seconds(OnOff_START_TIME)));
    client.SetAttribute("StopTime", TimeValue(Seconds(OnOff_STOP_TIME)));

    ApplicationContainer staApp;

    for (uint32_t i = 0; i < nSta; i++)
    {
        client.SetConstantRate(DataRate(coordsListUe[i].d), PacketSize);
        std::cout << coordsListUe[i].d << std::endl;
        // client.SetAttribute("PacketSize", UintegerValue(PacketSize));
        // client.SetAttribute("OnTime",
        // StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        // client.SetAttribute("OffTime",
        // StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        // client.SetAttribute("DataRate", DataRateValue(DataRate(dataRate)));

        staApp.Add(client.Install(wifiStaNodes.Get(i)));
    }

    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx",
                                  MakeCallback(&SendPacket));

  


    for (uint32_t nodeId = 0; nodeId < wifiStaNodes.GetN(); nodeId++)
    {
        // Get the ID of the node
        // uint32_t nodeId = node->GetId();
        Ptr<Node> node = wifiStaNodes.Get(nodeId);

        // Get the number of network interfaces on the node
        uint32_t nInterfaces = node->GetNDevices();

        // Print the node ID and number of interfaces
        std::cout << "Node " << nodeId << " has " << nInterfaces << " network interfaces"
                  << std::endl;
    };
    // double receiverSensitivity = -62;
    // Ptr<WifiPhyStateHelper> record =
    //     wifiPhyStateHelper.GetCurrentState(wifiStaNodes.Get(0)->GetDevice(0));
    // double rssi = record->rssiDbm;
    // double lossInDb = receiverSensitivity - rssi;
    // NS_LOG_INFO("Loss of link: " << lossInDb << " dB");
    if (tracing == true)
    {
        wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        wifiPhy.EnablePcap("Accesspoint", apDevice);
        wifiPhy.EnablePcap("station", staDevice);
    }

    std::ostringstream oss;
    oss << "/NodeList/" << wifiApNode.Get(0)->GetId() << "/ApplicationList/*/$ns3::PacketSink/Rx";
    Config::ConnectWithoutContext(oss.str(), MakeCallback(&ReceivePacket));
    // Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback(&ReceivePacket));


    AnimationInterface anim{"positioning.xml"};
    // FlowMonitorHelper flowmon;
    // Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Schedule(Seconds(Simulation_Start_Time), &CalculateThroughput);
    Simulator::Stop(Seconds(Simulation_Stop_Time + 1));
    Simulator::Run();

    // monitor->CheckForLostPackets();
    // monitor->SerializeToXmlFile("positioningflow5.xml", true, true);
    double averageThroughput =
        ((sink->GetTotalRx() * 8.0) / (1e6 * (Simulation_Stop_Time - Simulation_Start_Time)));

    if (averageThroughput < 10)
    {
        NS_LOG_ERROR("Obtained throughput is not in the expected boundaries!");
        exit(1);
    }
    std::cout << "\nAverage throughput: " << averageThroughput << " Mbit/s" << std::endl;
    std::cout << "Tx Bytes = " << txBytesCounter << std::endl;
    std::cout << "Rx Bytes = " << rxByteCounter << std::endl;

    Simulator::Destroy();
    return 0;
}
