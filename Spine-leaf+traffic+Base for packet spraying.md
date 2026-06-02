#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

int main(int argc, char *argv[])
{
    std::cout << "Spine-Leaf Packet Spraying Base Simulation Starting..." << std::endl;

    // -----------------------------
    // Create Nodes
    // -----------------------------
    NodeContainer hosts, leaves, spines;
    hosts.Create(2);   // Host0, Host1
    leaves.Create(2);  // Leaf0, Leaf1
    spines.Create(2);  // Spine0, Spine1

    // -----------------------------
    // Internet Stack
    // -----------------------------
    InternetStackHelper internet;
    internet.Install(hosts);
    internet.Install(leaves);
    internet.Install(spines);

    // -----------------------------
    // Point-to-Point Links
    // -----------------------------
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));

    // Host ↔ Leaf links
    NetDeviceContainer h0_l0 = p2p.Install(hosts.Get(0), leaves.Get(0));
    NetDeviceContainer h1_l1 = p2p.Install(hosts.Get(1), leaves.Get(1));

    // Leaf ↔ Spine links (multi-path)
    NetDeviceContainer l0_s0 = p2p.Install(leaves.Get(0), spines.Get(0));
    NetDeviceContainer l0_s1 = p2p.Install(leaves.Get(0), spines.Get(1));
    NetDeviceContainer l1_s0 = p2p.Install(leaves.Get(1), spines.Get(0));
    NetDeviceContainer l1_s1 = p2p.Install(leaves.Get(1), spines.Get(1));

    // -----------------------------
    // IP Addressing
    // -----------------------------
    Ipv4AddressHelper address;
    int subnet = 1;

    auto assign = [&](NetDeviceContainer devices)
    {
        std::ostringstream subnetBase;
        subnetBase << "10.1." << subnet++ << ".0";
        address.SetBase(subnetBase.str().c_str(), "255.255.255.0");
        return address.Assign(devices);
    };

    assign(h0_l0);
    assign(h1_l1);
    assign(l0_s0);
    assign(l0_s1);
    assign(l1_s0);
    assign(l1_s1);

    // -----------------------------
    // Routing
    // -----------------------------
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // -----------------------------
    // SERVER (Receiver at Host1)
    // -----------------------------
    UdpEchoServerHelper server(9);
    ApplicationContainer serverApp = server.Install(hosts.Get(1));
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(20.0));

    // -----------------------------
    // CLIENT (Sender from Host0)
    // -----------------------------
    UdpEchoClientHelper client(Ipv4Address("10.1.2.2"), 9);
    client.SetAttribute("MaxPackets", UintegerValue(20));
    client.SetAttribute("Interval", TimeValue(Seconds(0.3)));
    client.SetAttribute("PacketSize", UintegerValue(512));

    ApplicationContainer clientApp = client.Install(hosts.Get(0));
    clientApp.Start(Seconds(2.0));
    clientApp.Stop(Seconds(20.0));

    // -----------------------------
    // Run Simulation
    // -----------------------------
    std::cout << "Running simulation..." << std::endl;

    Simulator::Run();
    Simulator::Destroy();

    std::cout << "Simulation finished." << std::endl;

    return 0;
}
