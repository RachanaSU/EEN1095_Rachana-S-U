# EEN1095_Rachana-S-U
A00048147-MECE-Engineering Final Project 2025-26
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include <iostream>

using namespace ns3;


int main(int argc, char *argv[])
{
    std::cout << " SIMULATION STARTED " << std::endl;

    NodeContainer hosts, leaves, spines;

    hosts.Create(2);
    leaves.Create(2);
    spines.Create(2);

    InternetStackHelper internet;
    internet.Install(hosts);
    internet.Install(leaves);
    internet.Install(spines);

    PointToPointHelper link;
    link.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    link.SetChannelAttribute("Delay", StringValue("2ms"));

    // Store devices for IP assignment
    NetDeviceContainer h1_l1 = link.Install(hosts.Get(0), leaves.Get(0));
    NetDeviceContainer h2_l2 = link.Install(hosts.Get(1), leaves.Get(1));

    NetDeviceContainer l1_s1 = link.Install(leaves.Get(0), spines.Get(0));
    NetDeviceContainer l1_s2 = link.Install(leaves.Get(0), spines.Get(1));
    NetDeviceContainer l2_s1 = link.Install(leaves.Get(1), spines.Get(0));
    NetDeviceContainer l2_s2 = link.Install(leaves.Get(1), spines.Get(1));

    Ipv4AddressHelper address;
    int subnet = 1;

    auto assign = [&](NetDeviceContainer d) {
        std::string base = "10.1." + std::to_string(subnet++) + ".0";
        address.SetBase(base.c_str(), "255.255.255.0");
        return address.Assign(d);
    };

    assign(h1_l1);
    assign(h2_l2);
    assign(l1_s1);
    assign(l1_s2);
    assign(l2_s1);
    assign(l2_s2);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // UDP Receiver (Host2)
    UdpEchoServerHelper server(9);
    ApplicationContainer serverApp = server.Install(hosts.Get(1));
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(20.0));

    // UDP Sender (Host1)
    UdpEchoClientHelper client(Ipv4Address("10.1.2.2"), 9);
    client.SetAttribute("MaxPackets", UintegerValue(10));
    client.SetAttribute("Interval", TimeValue(Seconds(0.5)));
    client.SetAttribute("PacketSize", UintegerValue(512));

    ApplicationContainer clientApp = client.Install(hosts.Get(0));
    clientApp.Start(Seconds(2.0));
    clientApp.Stop(Seconds(20.0));

    std::cout << " Running simulation..." << std::endl;

    Simulator::Run();
    Simulator::Destroy();

    std::cout << " SIMULATION ENDED" << std::endl;

    return 0;
}
