/******************************************************************************
*                        (c) Copyright INESC TEC, 2020.
*            All rights reserved. Content distribution not allowed.
*******************************************************************************

*******************************************************************************/

// Modules headers

// ns-3
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4.h"
#include "ns3/mesh-helper.h"
#include "ns3/mesh-module.h"
#include "ns3/minstrel-ht-wifi-manager.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-module.h"
#include "ns3/snr-tag.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-module.h"

using namespace ns3;

// ==================================================================
//           CONSTANTS
// ==================================================================

// Number of FMAPS
#define N_FMAPS 4

// Number of Gateway UAVs
#define N_GATEWAY_UAVS 1

// Transport protocol (true for UDP; false for TCP)
#define UDP true

// Rician K-factor (in dB)
#define RICIAN_K_FACTOR_DB 13

// Define time interval (in seconds) to collect samples for delay results
#define DELAY_SAMPLE_INTERVAL 0.01

// Packet size (bytes)
#define PACKET_SIZE 1400

// Queue Management Algorithm
#define QUEUE_MANAGEMENT_ALGORITHM "CoDel" // To avoid "bufferbloat"

// Queue default size
#define QUEUE_SIZE "1000p" // 1000 packets = default value

// Upper bound for the Random Number Generator
#define RNG_UPPER_BOUND 1

// IEEE 802.11 channel number
#define CHANNEL_NUMBER 50

// IEEE 802.11 channel bandwidth (MHz)
#define CHANNEL_BW 160

// IPv4 Network
#define NETWORK "10.0.0.0"

// IPV4 netmask
#define NETMASK "255.255.255.0"

// Number of mesh interfaces (if applicable)
#define N_INTERFACES 1

// Avoid starting all mesh nodes at the same time, since beacons may collide (if applicable)
#define RANDOM_START 0.3

// ----- SIMULATION TIMES ----- //

// Applications time
#define APPS_DURATION_TIME 130 // Applications duration time (in seconds)

#define APPS_SERVER_START_TIME                                                                     \
    30 // Server applications start time (in seconds) -> != 0 to allow the network to exchange OLSR
       // messages
#define APPS_CLIENT_START_TIME                                                                     \
    (APPS_SERVER_START_TIME + 1) // Client applications start time (in seconds) -> Should start
                                 // after the server has initialized

#define FLOWMONITOR_START_TIME APPS_CLIENT_START_TIME // FlowMonitor's capture start time

#define APPS_CLIENT_STOP_TIME                                                                      \
    (FLOWMONITOR_START_TIME + APPS_DURATION_TIME) // Client applications stop time (in seconds)
#define APPS_SERVER_STOP_TIME                                                                      \
    (APPS_CLIENT_STOP_TIME + 2) // Server applications stop time (in seconds)

// Global simulation
#define SIMULATION_END_TIME (APPS_SERVER_STOP_TIME + 1) // Simulation end time (in seconds)

// ----- APPLICATIONS ----- //
#define SERVER_APP_PORT 9 // Server app port

// ----- FILENAMES ----- //
#define PCAP_BASE_FILENAME "gateway-placement-dev" // Base filename of the output PCAP file

// ==================================================================
//           LOGGING
// ==================================================================
NS_LOG_COMPONENT_DEFINE("GatewayPlacement");

/**
 * Enable all simulation logging components.
 *
 * \param logLevel   Log level to use.
 */

void
EnableLog(LogLevel logLevel)
{
    LogComponentEnableAll(LOG_LEVEL_INFO);
    EnableLogComponent("GatewayPlacement", logLevel);
    EnableLogComponent("Util", logLevel);
    EnableLogComponent("OnOffApplication", logLevel);
    EnableLogComponent("PacketSink", logLevel);
    EnableLogComponent("ArpL3Protocol", LOG_LEVEL_LOGIC);
}

// ==================================================================
//           GLOBAL VARIABLES
// ==================================================================

double rxByteCounter = 0;
double oldRxByteCounter = 0;
double rxPacketCounter = 0;
double oldRxPacketCounter = 0;
double txPacketCounter = 0;
double oldTxPacketCounter = 0;
double rxPacketSum = 0;
double txPacketSum = 0;
double throughput = 0;
Time delayCounter;
Time oldDelayCounter;
Time delaySum;
double previousSimulationTime = 0; // Reference for the delay sample interval
double trafficDemand[N_FMAPS];

std::ofstream averageQosResults; // Output file to save the results for throughput, PDR, and delay,
                                 // considering average values measured at each second

std::ofstream delayResults; // Output file to save the delay results, considering each packet
                            // received at each instant defined by the delaySampleInterval variable

Ptr<PacketSink> packetSink; // Pointer to the packet sink application */

Ptr<PoissonTrafficGenerator>* app =
    new Ptr<PoissonTrafficGenerator>[N_FMAPS]; /* Pointer to the Poisson traffic generator
                                                  application */

void
ShowPosition(Ptr<Node> node, double deltaTime)
{
    uint32_t nodeId = node->GetId();
    Ptr<MobilityModel> mobModel = node->GetObject<MobilityModel>();
    Vector3D pos = mobModel->GetPosition();
    Vector3D speed = mobModel->GetVelocity();
    std::cout << "At " << Simulator::Now().GetSeconds() << " node " << nodeId << ": Position("
              << pos.x << ", " << pos.y << ", " << pos.z << ");   Speed(" << speed.x << ", "
              << speed.y << ", " << speed.z << ")" << std::endl;

    Simulator::Schedule(Seconds(deltaTime), &ShowPosition, node, deltaTime);
}

// QoS metrics over time
void
QoSMonitor(int nTxNodes)
{
    // bytes received
    throughput = rxByteCounter - oldRxByteCounter;
    if (UDP)
    {
        txPacketCounter = 0;
        for (int i = 0; i < nTxNodes; i++)
        {
            // number of packets sent
            txPacketCounter += app[i]->m_packetsSent;
        }
    }

    // number of packets received
    rxPacketCounter = rxByteCounter / PACKET_SIZE;

    rxPacketSum = rxPacketCounter - oldRxPacketCounter;
    txPacketSum = txPacketCounter - oldTxPacketCounter;
    averageQosResults << Simulator::Now().GetSeconds() << ";" << throughput * 8 / 1e6 << ";"
                      << rxPacketSum / txPacketSum << ";"
                      << (delayCounter.GetNanoSeconds() - oldDelayCounter.GetNanoSeconds()) /
                             rxPacketSum
                      << std::endl;
    oldRxByteCounter = rxByteCounter;
    oldRxPacketCounter = rxPacketCounter;
    oldTxPacketCounter = txPacketCounter;
    oldDelayCounter = delayCounter;
    Simulator::Schedule(Seconds(1), &QoSMonitor, nTxNodes);
}

// Count received bytes and delay
void
ReceivePacket(Ptr<const Packet> p, const Address& addr)
{
    SnrTag snrTag;
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

    if (p->FindFirstMatchingByteTag(timestampTag))
    {
        timeTx = timestampTag.GetTimestamp();
        delayCounter += (Simulator::Now() - timeTx);

        if (Simulator::Now().GetSeconds() >= (previousSimulationTime + DELAY_SAMPLE_INTERVAL))
        {
            delayResults << Simulator::Now().GetSeconds() << ";"
                         << Simulator::Now().GetNanoSeconds() - timeTx.GetNanoSeconds()
                         << std::endl;
            previousSimulationTime = Simulator::Now().GetSeconds();
        }
    }
    rxByteCounter += p->GetSize();
    //}
}

// Packets Sent by BulkSendApplication
void
SendPacket(Ptr<const Packet> pkt)
{
    txPacketCounter++;
}

// Set NIC TX power
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

// Set antenna TX gain
void
SetTxGain(int nodeID, double value)
{
    std::ostringstream oss;
    oss << "/NodeList/" << nodeID << "/DeviceList/*/$ns3::WifiNetDevice/Phy/TxGain";
    Config::Set(oss.str(), DoubleValue(value));
}

// Set antenna RX gain
void
SetRxGain(int nodeID, double value)
{
    std::ostringstream oss;
    oss << "/NodeList/" << nodeID << "/DeviceList/*/$ns3::WifiNetDevice/Phy/RxGain";
    Config::Set(oss.str(), DoubleValue(value));
}

// Configure nodes (tx power, antenna gains, and traffic demand), based on the information in
// nodesInformationFilename file
void
ConfigureNodes(std::string nodesInformationFilename, int& nTxNodes)
{
    std::string line;
    std::string field;
    std::vector<std::string> result;
    std::ifstream nodesInformationFile(nodesInformationFilename);

    if (!nodesInformationFile.is_open())
    {
        std::cout << nodesInformationFilename << " file does not exist." << std::endl;
        return;
    }
    std::getline(nodesInformationFile, line); // skip the first line (header)
    for (int i = 0; std::getline(nodesInformationFile, line); i++)
    {
        std::stringstream ss(line);
        while (ss.good())
        {
            getline(ss, field, ',');
            result.push_back(field);
        }

        SetTxPower(std::stoi(result[1].c_str(), 0), strtof(result[2].c_str(), 0));

        SetTxGain(std::stoi(result[1].c_str(), 0), strtof(result[3].c_str(), 0));

        SetRxGain(std::stoi(result[1].c_str(), 0), strtof(result[4].c_str(), 0));

        trafficDemand[std::stoi(result[1].c_str(), 0)] = strtof(result[5].c_str(), 0);

        if (strtof(result[5].c_str(), 0) > 0)
            nTxNodes++;

        /*if (std::stoi(result[8].c_str(), 0) == 1)
        {
            mobility.Install(fbmn.Get(std::stoi(result[1].c_str(), 0)));
            setNodePosition(fbmn.Get(std::stoi(result[1].c_str(), 0)),
        Vector(strtof(result[2].c_str(), 0), strtof(result[3].c_str(), 0), strtof(result[4].c_str(),
        0))); Simulator::Schedule(Seconds(APPS_SERVER_START_TIME), &SetVelocity,
        fbmn.Get(std::stoi(result[1].c_str(), 0)), Vector(strtof(result[5].c_str(), 0),
        strtof(result[6].c_str(), 0), strtof(result[7].c_str(), 0)));
        }*/
        result.erase(result.begin(), result.end());
    }

    if (nodesInformationFile.is_open())
    {
        nodesInformationFile.close();
    }
}

// Add route
void
AddRoute(Ptr<Ipv4StaticRouting> staticRouting,
         Ipv4Address nextHop,
         Ipv4Address destination,
         int ifIndex)
{
    staticRouting->AddHostRouteTo(destination, nextHop, ifIndex);
    // std::cout << Simulator::Now().GetSeconds() << "Route added -> Destination: " << destination
    // << " Next hop: " << nextHop << std::endl;
}

// Remove route
void
RemoveRoute(Ptr<Ipv4StaticRouting> staticRouting, Ipv4Address destination)
{
    for (uint32_t i = 0; i < staticRouting->GetNRoutes(); i++)
    {
        if (staticRouting->GetRoute(i).GetDest() == destination)
        {
            staticRouting->RemoveRoute(i);
            // std::cout << Simulator::Now().GetSeconds() << "Route removed -> Destination: " <<
            // destination << std::endl;
        }
    }
}

// Configure forwarding tables based on the information in routingFilename file
void
RoutingScheduler(std::string routingFilename,
                 Ptr<Ipv4StaticRouting>* staticRoutingList,
                 Ipv4Address* ipv4AddressList,
                 std::string routingProtocol)
{
    std::string line;
    std::string field;
    std::vector<std::string> result;
    std::ifstream routingFile(routingFilename);
    if (!routingFile.is_open())
    {
        std::cout << routingFilename << " file does not exist." << std::endl;
        exit(0);
    }
    std::getline(routingFile, line); // skip the first line (header)
    for (int i = 0; std::getline(routingFile, line); i++)
    {
        std::stringstream ss(line);
        while (ss.good())
        {
            getline(ss, field, ',');
            result.push_back(field);
        }
        if (i == 0) // first step
        {
            for (int j = 0; j < (N_FMAPS + N_GATEWAY_UAVS); j++)
                Simulator::Schedule(Seconds(0),
                                    &RemoveRoute,
                                    staticRoutingList[j],
                                    NETWORK); // remove default route
        }
        Simulator::Schedule(Seconds(strtof(result[0].c_str(), 0)),
                            &RemoveRoute,
                            staticRoutingList[std::stoi(result[1].c_str(), 0)],
                            v4AddressList[N_FMAPS]);
        Simulator::Schedule(Seconds(strtof(result[0].c_str(), 0)),
                            &RemoveRoute,
                            staticRoutingList[N_FMAPS],
                            ipv4AddressList[std::stoi(result[1].c_str(), 0)]);

        Simulator::Schedule(Seconds(strtof(result[0].c_str(), 0)),
                            &AddRoute,
                            staticRoutingList[std::stoi(result[1].c_str(), 0)],
                            ipv4AddressList[std::stoi(result[2].c_str(), 0)],
                            ipv4AddressList[N_FMAPS],
                            1);
        if ((std::stoi(result[2].c_str(), 0)) == N_FMAPS)
            Simulator::Schedule(Seconds(strtof(result[0].c_str(), 0)),
                                &AddRoute,
                                staticRoutingList[N_FMAPS],
                                ipv4AddressList[std::stoi(result[1].c_str(), 0)],
                                ipv4AddressList[std::stoi(result[1].c_str(), 0)],
                                1);
        else
        {
            Simulator::Schedule(Seconds(strtof(result[0].c_str(), 0)),
                                &AddRoute,
                                staticRoutingList[N_FMAPS],
                                ipv4AddressList[std::stoi(result[2].c_str(), 0)],
                                ipv4AddressList[std::stoi(result[1].c_str(), 0)],
                                1);
        }
        result.erase(result.begin(), result.end());
    }
    if (routingFile.is_open())
    {
        routingFile.close();
    }
}

// Set Waypoints defined in scenarioFilename
static void
SetWaypoints(NodeContainer& nodes, std::string scenarioFilename)
{
    Waypoint wpt;
    std::vector<std::string> nodeInformation;
    std::string line;
    std::string field;
    Ptr<WaypointMobilityModel> mob;
    std::ifstream scenarioFile(scenarioFilename);
    int index;
    if (!scenarioFile.is_open())
    {
        std::cout << scenarioFilename << " file does not exist." << std::endl;
        exit(0);
    }

    std::getline(scenarioFile, line);        // skip the first line (header)
    while (std::getline(scenarioFile, line)) // iterate over the file lines
    {
        std::stringstream ss(line);
        while (ss.good()) // get the information fields from each line
        {
            std::getline(ss, field, ',');
            nodeInformation.push_back(field);
        }

        index = atoi(nodeInformation[0].c_str());
        mob = nodes.Get(index)->GetObject<WaypointMobilityModel>();
        wpt = Waypoint(Seconds(strtof(nodeInformation[1].c_str(), 0)),
                       Vector(strtof(nodeInformation[2].c_str(), 0),
                              strtof(nodeInformation[3].c_str(), 0),
                              strtof(nodeInformation[4].c_str(), 0)));
        mob->AddWaypoint(wpt);

        nodeInformation.erase(nodeInformation.begin(),
                              nodeInformation.end()); // clear nodeInformation variable
    }
    if (scenarioFile.is_open())
    {
        scenarioFile.close();
    }
}

// ==================================================================
//           MAIN
// ==================================================================
int
main(int argc, char* argv[])
{
    // ==================================================================
    //           INITIAL SETUP
    // ==================================================================
    NS_LOG_UNCOND("=================================================="
                  << std::endl
                  << "GATEWAY PLACEMENT STUDY" << std::endl
                  << "==================================================" << std::endl);

    // ==================== GLOBAL VARIABLES ==================== //

    // Simulation parameters
    std::string routingProtocol = "";
    int nTxNodes = 0; // Number of traffic sources
    // double deltaTime = 1;                   //Interval in which position of nodes is shown
    std::string queueManagementAlgorithm; // Queue management discipline

    // Input filenames
    std::string nodesInformationFilename;
    std::string scenarioFilename;
    std::string routingFilename;
    std::string queueManagementFilename;

    // Output / log

    // QoS results filename
    std::string resultsFilename; // Filename of the output results file

    bool isLogEnabled = false;                  // Flag to enable log
    std::string logLevelStr = "LOG_LEVEL_INFO"; // Log level to use in this simulation

    // FlowMonitor filename
    std::string flowMonitorFilename;               // Filename of the output FlowMonitor file
    std::string pcapFilename = PCAP_BASE_FILENAME; // Filename of the output PCAP file

    // Random Number Generators -> To allow independent simulations, in order to obtain confidence
    // intervals
    uint64_t rngSeed = 0; // Initialized with zero to be able to check if this variable has been set
    uint64_t nSimulation = 0;

    // ==================== COMMAND-LINE ARGUMENTS ==================== //
    CommandLine cmd;

    // Simulation parameters
    cmd.AddValue("routingProtocol",
                 "Routing protocol. Possible values: \"AODV\", \"OLSR\", \"Mesh\", \"RedeFINE\".",
                 routingProtocol);

    // Enable log
    cmd.AddValue("enableLog", "Enable logging components", isLogEnabled);
    cmd.AddValue("logLevel",
                 "Log level (format: \"LOG_LEVEL_<LEVEL>\" or \"<LEVEL>\")",
                 logLevelStr);

    // FlowMonitor filename
    cmd.AddValue("flowMonitorFilename",
                 "Filename of the output Flow Monitor file (without \".xml\")",
                 flowMonitorFilename);

    // Nodes' information filename
    cmd.AddValue("nodesInformationFilename",
                 "Filename of the nodes' information file",
                 nodesInformationFilename);

    // Scenario filename
    cmd.AddValue("scenarioFilename", "Filename of the scenario input file", scenarioFilename);

    // Routing interference filename
    cmd.AddValue("routingFilename", "Filename of the routing input file", routingFilename);

    // Results filename
    cmd.AddValue("resultsFilename", "Filename of the output results file", resultsFilename);

    // Random Number Generators -> To allow independent simulations, in order to obtain confidence
    // intervals
    cmd.AddValue("rngSeed",
                 "Random Number Generator seed (to define a deterministic simulation)",
                 rngSeed);
    cmd.AddValue("nSimulation",
                 "Number of the simulation run (for confidence intervals)",
                 nSimulation);

    cmd.Parse(argc, argv);

    // ==================== LOGGING ==================== //
    if (isLogEnabled)
    {
        EnableLog(ParseLogLevelString(logLevelStr));
    }

    // ==================== VALIDATE ARGUMENTS ==================== //

    // Check if the simulation parameters are valid
    if (routingProtocol.empty() || (routingProtocol != "AODV" && routingProtocol != "OLSR" &&
                                    routingProtocol != "Mesh" && routingProtocol != "RedeFINE"))
    {
        cmd.PrintHelp(std::cout);
        NS_ABORT_MSG("Simulation parameters invalid!");
    }

    // Define RNG seeds
    if (rngSeed != 0)
    {
        NS_LOG_INFO(">> RNG SEED DEFINED: " << rngSeed);
        RngSeedManager::SetSeed(rngSeed);
    }

    if (nSimulation != 0)
    {
        NS_LOG_INFO(">> SIMULATION NUMBER DEFINED: " << nSimulation);
        RngSeedManager::SetRun(nSimulation);
    }

    // ==================================================================
    //           SCENARIO SETUP
    // ==================================================================

    // FMAPs
    NodeContainer fmaps;
    fmaps.Create(N_FMAPS);

    // Gateway UAVs
    NodeContainer gatewayUavs;
    gatewayUavs.Create(N_GATEWAY_UAVS);

    // FMAPs + Gateway UAVs
    NodeContainer fbmn(fmaps, gatewayUavs);

    // Log
    if (isLogEnabled)
    {
        logNodesPositions(fmaps, "FMAPs POSITIONS");
        logNodesPositions(gatewayUavs, "GW UAVs POSITIONS");
    }

    // ==================== WI-FI SETUP ==================== //

    // Wi-Fi standard and frequency
    const enum WifiPhyStandard wifiStandard =
        WIFI_PHY_STANDARD_80211ac; // Wi-Fi standard (WIFI_PHY_STANDARD_80211ac) // Wi-Fi channel
                                   // number (50 @ 5250 MHz)
    double wifiFrequencyHz =
        determineWifiChannelFrequencyHz(wifiStandard, CHANNEL_NUMBER); // Absolute frequency (in Hz)

    // Wi-Fi channel
    YansWifiChannelHelper wifiChannel;

    // Friis propagation loss model
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel",
                                   "Frequency",
                                   DoubleValue(wifiFrequencyHz));

    // Rician component modeled by the Nakagami distribution (it constitutes an approximation); a
    // single "m" is defined UNCOMMENT THE FOLLOWING LINES TO ENABLE THE RICIAN COMPONENT
    /*****
    wifiChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel",
                                   "Distance1", DoubleValue(0),
                                   "Distance2", DoubleValue(0),
                                   "m0", DoubleValue(0),
                                   "m1", DoubleValue(0),
                                   "m2",
    DoubleValue(RicianKFactor2NakagamiMParameter(RICIAN_K_FACTOR_DB)));
    *****/

    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    // Wi-Fi PHY
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    wifiPhy.SetPcapDataLinkType(
        YansWifiPhyHelper::DLT_IEEE802_11_RADIO); // Enable radio-tap information on PCAP files
    wifiPhy.Set("ChannelNumber", UintegerValue(CHANNEL_NUMBER));
    wifiPhy.SetChannel(wifiChannel.Create());
    wifiPhy.Set("TxPowerLevels", UintegerValue(1));
    wifiPhy.Set("ShortGuardEnabled", BooleanValue(false));

    // Wi-Fi MAC
    WifiMacHelper wifiMac;

    if (routingProtocol != "Mesh")
    {
        Ssid ssid = Ssid("FBMN");
        wifiMac.SetType("ns3::AdhocWifiMac",
                        "QosSupported",
                        BooleanValue(true),
                        "Ssid",
                        SsidValue(ssid));
    }

    // Wi-Fi
    WifiHelper wifi;
    wifi.SetStandard(wifiStandard);
    wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
    // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
    // StringValue("VhtMcs0"), "ControlMode", StringValue("VhtMcs0"));
    // Config::SetDefault("WifiRemoteStationManager::FragmentationThreshold", StringValue("2200"));
    // Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("100"));

    NetDeviceContainer netDevices;
    if (routingProtocol == "Mesh")
    {
        std::string root = "ff:ff:ff:ff:ff:ff";
        MeshHelper mesh = MeshHelper::Default();
        if (!Mac48Address(root.c_str()).IsBroadcast())
        {
            mesh.SetStackInstaller("ns3::Dot11sStack",
                                   "Root",
                                   Mac48AddressValue(Mac48Address(root.c_str())));
        }
        else
        {
            // If root is not set, we do not use "Root" attribute, because it is specified only for
            // 11s
            mesh.SetStackInstaller("ns3::Dot11sStack");
        }

        // Set number of interfaces - default is single-interface mesh point
        mesh.SetNumberOfInterfaces(N_INTERFACES);

        netDevices = mesh.Install(wifiPhy, fbmn);
    }

    if (routingProtocol != "Mesh")
        netDevices = wifi.Install(wifiPhy, wifiMac, fbmn);

    // Channel BW
    Config::Set("/NodeList/*/DeviceList/*/F/Phy/ChannelWidth", UintegerValue(CHANNEL_BW));

    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    // ==================== ROUTING ==================== //
    AodvHelper aodv;
    OlsrHelper olsr;
    Ipv4StaticRoutingHelper staticRouting;
    Ipv4ListRoutingHelper listRouting;

    if (routingProtocol == "AODV")
    {
        listRouting.Add(aodv, 0);
    }
    else if (routingProtocol == "OLSR")
    {
        listRouting.Add(olsr, 0);
    }

    // ==================== INTERNET STACK ==================== //
    InternetStackHelper internetStack;
    if (routingProtocol != "RedeFINE" && routingProtocol != "Mesh")
        internetStack.SetRoutingHelper(listRouting);
    internetStack.Install(fbmn);

    // ==================== IP ADDRESSES ==================== //
    Ipv4AddressHelper ipv4Address;
    ipv4Address.SetBase(NETWORK, NETMASK);

    Ipv4InterfaceContainer Ipv4Interfaces = ipv4Address.Assign(netDevices);

    Ptr<Ipv4>* ipv4List = new Ptr<Ipv4>[N_FMAPS + N_GATEWAY_UAVS];
    Ptr<Ipv4StaticRouting>* staticRoutingList =
        new Ptr<Ipv4StaticRouting>[N_FMAPS + N_GATEWAY_UAVS];
    Ipv4Address* ipv4AddressList = new Ipv4Address[N_FMAPS + N_GATEWAY_UAVS];
    for (int i = 0; i < N_FMAPS + N_GATEWAY_UAVS; i++)
    {
        ipv4List[i] = fbmn.Get(i)->GetObject<Ipv4>();
        staticRoutingList[i] = staticRouting.GetStaticRouting(ipv4List[i]);
        ipv4AddressList[i] = ipv4List[i]->GetAddress(1, 0).GetLocal();
    }

    // ====== NODES CONFIGURATION (TX POWER, ANTENNA GAINS, TRAFFIC DEMAND) ====== //

    ConfigureNodes(nodesInformationFilename, nTxNodes);

    MobilityHelper mobilityFmaps; // helper using waypoint mobility model for dynamic scenarios
    Ptr<ListPositionAllocator> positionAllocFmaps = CreateObject<ListPositionAllocator>();
    positionAllocFmaps->Add(Vector(0, 0, 0));

    mobilityFmaps.SetPositionAllocator(positionAllocFmaps);
    mobilityFmaps.SetMobilityModel("ns3::WaypointMobilityModel",
                                   "InitialPositionIsWaypoint",
                                   BooleanValue(false));
    mobilityFmaps.Install(fbmn);

    SetWaypoints(fbmn, scenarioFilename);

    // ==================================================================
    //           APPLICATIONS
    // ==================================================================

    // UDP sink
    if (UDP)
    {
        PacketSinkHelper packetSink =
            PacketSinkHelper("ns3::UdpSocketFactory",
                             InetSocketAddress(Ipv4Address::GetAny(), SERVER_APP_PORT));
        packetSink.SetAttribute("StartTime", TimeValue(Seconds(APPS_SERVER_START_TIME)));
        packetSink.SetAttribute("StopTime", TimeValue(Seconds(APPS_SERVER_STOP_TIME)));

        ApplicationContainer packetSinkApps;
        packetSinkApps = packetSink.Install(fbmn.Get(N_FMAPS)); // sink at Gateway UAV
    }

    // Set UDP Poisson traffic generator
    if (UDP)
    {
        // SettrafficDemand(nodesInformationFilename);
        Ptr<Socket>* ns3UdpSocket = new Ptr<Socket>[N_FMAPS];
        for (int i = 0; i < nTxNodes; i++)
        {
            app[i] = CreateObject<PoissonTrafficGenerator>();
            ns3UdpSocket[i] = Socket::CreateSocket(fmaps.Get(i), UdpSocketFactory::GetTypeId());
            fbmn.Get(i)->AddApplication(app[i]);
            app[i]->Setup(ns3UdpSocket[i],
                          InetSocketAddress(Ipv4Interfaces.GetAddress(N_FMAPS), SERVER_APP_PORT),
                          PACKET_SIZE,
                          trafficDemand[i] / (PACKET_SIZE * 8),
                          RNG_UPPER_BOUND);
            // std::cout << "Traffic load: " << trafficDemand[i] / (PACKET_SIZE * 8) << std::endl;
            app[i]->SetStopTime(Seconds(APPS_CLIENT_STOP_TIME));
            app[i]->SetStartTime(Seconds(APPS_CLIENT_START_TIME));
        }
    }
    // Set TCP BulkSend traffic generator
    else
    {
        ApplicationContainer sourceApp[N_FMAPS];
        Address sinkLocalAddress(InetSocketAddress(Ipv4Address::GetAny(), SERVER_APP_PORT));
        PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", sinkLocalAddress);
        AddressValue remoteAddress(
            InetSocketAddress(Ipv4Interfaces.GetAddress(N_FMAPS), SERVER_APP_PORT));
        for (int i = 0; i < nTxNodes; i++)
        {
            BulkSendHelper ftp("ns3::TcpSocketFactory", Address());
            ftp.SetAttribute("Remote", remoteAddress);
            ftp.SetAttribute("SendSize", UintegerValue(PACKET_SIZE));
            ftp.SetAttribute("MaxBytes", UintegerValue(0));
            sourceApp[i] = ftp.Install(fmaps.Get(i));

            sourceApp[i].Start(Seconds(APPS_CLIENT_START_TIME));
            sourceApp[i].Stop(Seconds(APPS_CLIENT_STOP_TIME));
        }

        sinkHelper.SetAttribute("Protocol", TypeIdValue(TcpSocketFactory::GetTypeId()));
        ApplicationContainer sinkApp = sinkHelper.Install(fbmn.Get(N_FMAPS));
        sinkApp.Start(Seconds(APPS_SERVER_START_TIME));
        sinkApp.Stop(Seconds(APPS_SERVER_STOP_TIME));

        packetSink = StaticCast<PacketSink>(sinkApp.Get(0));

        // Trace sent packets
        Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::BulkSendApplication/Tx",
                                      MakeCallback(&SendPacket));
    }

    // ===================================================
    //                     CONFIGURE
    // ===================================================

    // ==================== ROUTING ==================== //

    if (routingProtocol == "RedeFINE")
        RoutingScheduler(routingFilename, staticRoutingList, ipv4AddressList, routingProtocol);

    // Trace routing tables
    // Ipv4GlobalRoutingHelper globalRoutingHelper;
    // Ptr<OutputStreamWrapper> routingStream =
    // Create<OutputStreamWrapper>("dynamic-global-routing.routes", std::ios::out);
    // globalRoutingHelper.PrintRoutingTableAllEvery(Seconds(1), routingStream); //change me

    // ==================== QUEUING ==================== //

    TrafficControlHelper
        tch; /* Traffic Control. Do not use when the network is in mesh configuration. */
    Ptr<NetDevice>* dev = new Ptr<NetDevice>[N_FMAPS];
    Ptr<WifiNetDevice>* wifi_dev = new Ptr<WifiNetDevice>[N_FMAPS];
    QueueDiscContainer qdiscs;

    if (strcmp(QUEUE_MANAGEMENT_ALGORITHM, "CoDel") == 0)
    {
        Config::SetDefault("ns3::CoDelQueueDisc::MaxSize", StringValue(QUEUE_SIZE));
        tch.SetRootQueueDisc("ns3::CoDelQueueDisc");

        for (int j = 0; j < N_FMAPS; j++)
        {
            dev[j] = fbmn.Get(j)->GetDevice(0);
            wifi_dev[j] = DynamicCast<WifiNetDevice>(dev[j]);
            qdiscs.Add(tch.Install(wifi_dev[j]));
        }
    }

    // ==================== TRACING ==================== //

    // Trace Received Packets
    std::ostringstream oss;
    oss << "/NodeList/" << N_FMAPS << "/ApplicationList/*/$ns3::PacketSink/Rx";
    Config::ConnectWithoutContext(oss.str(), MakeCallback(&ReceivePacket));

    // Trace devices (pcap)
    // wifiPhy.EnablePcap(pcapFilename, netDevices); ///change me

    // Configure Flow Monitor
    FlowMonitorHelper flowMonitorHelper;
    flowMonitorHelper.SetMonitorAttribute(
        "StartTime",
        TimeValue(Seconds(
            FLOWMONITOR_START_TIME))); // The FlowMonitor should start monitoring when the network
                                       // is stabilized (i.e., in steady-state regime)

    Ptr<FlowMonitor> flowMonitor = flowMonitorHelper.InstallAll();

    // Run simulation
    NS_LOG_INFO(">> SIMULATION STARTED" << std::endl);

    averageQosResults.open(resultsFilename, std::ios_base::out);
    averageQosResults << "Simulation time (s);Throughput (Mbit/s);Packet Delivery Ratio;Delay (ns)"
                      << std::endl; // write header to the averageQosResults file

    delayResults.open(resultsFilename + "-Delay", std::ios_base::out);
    delayResults << "Simulation time (s);Delay (ns)"
                 << std::endl; // write header to the delayResults file

    // Calculate Throughput using my QoSMonitor
    Simulator::Schedule(Seconds(FLOWMONITOR_START_TIME), &QoSMonitor, nTxNodes);

    Simulator::Stop(Seconds(SIMULATION_END_TIME));

    // Print UAVs positions at each second
    // for (int i = 0; i < (N_FMAPS + N_GATEWAY_UAVS); i++)
    // Simulator::Schedule(Seconds(0.0), &ShowPosition, fbmn.Get(i), 1);

    Simulator::Run();

    // Print Flow Monitor Results
    flowMonitor->CheckForLostPackets();

    // flowMonitor->SerializeToXmlFile((flowMonitorFilename + ".xml"), false, false);
    // PrintFlowMonitorResults(flowMonitorHelper, std::cout);

    // Finish simulation
    Simulator::Destroy();
    NS_LOG_INFO(">> SIMULATION ENDED" << std::endl);

    NS_LOG_UNCOND(std::endl << "==================== SIMULATION FINISHED ====================");

    if (averageQosResults.is_open())
    {
        averageQosResults.close();
    }
    if (delayResults.is_open())
    {
        delayResults.close();
    }
    return 0;
}
