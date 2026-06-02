#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

int main(int argc, char *argv[])
{
    std::cout << "Spine-Leaf Simulation Starting..." << std::endl;

    // 2 hosts, 2 leaf switches, 2 spine switches
    NodeContainer hosts, leaves, spines;
    hosts.Create(2);
    leaves.Create(2);
    spines.Create(2);

    InternetStackHelper internet;
    internet.Install(hosts);
    internet.Install(leaves);
    internet.Install(spines);

    PointToPointHelper link;
    link.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    link.SetChannelAttribute("Delay", StringValue("2ms"));

    NetDeviceContainer h1_l1 = link.Install(hosts.Get(0), leaves.Get(0));
    NetDeviceContainer h2_l2 = link.Install(hosts.Get(1), leaves.Get(1));

    NetDeviceContainer l1_s1 = link.Install(leaves.Get(0), spines.Get(0));
    NetDeviceContainer l1_s2 = link.Install(leaves.Get(0), spines.Get(1));
    NetDeviceContainer l2_s1 = link.Install(leaves.Get(1), spines.Get(0));
    NetDeviceContainer l2_s2 = link.Install(leaves.Get(1), spines.Get(1));

    Ipv4AddressHelper address;
    int subnet = 1;

    auto assign = [&](NetDeviceContainer d) {
        std::ostringstream subnetBase;
        subnetBase << "10.1." << subnet++ << ".0";
        address.SetBase(subnetBase.str().c_str(), "255.255.255.0");
        return address.Assign(d);
    };

    assign(h1_l1);
    assign(h2_l2);
    assign(l1_s1);
    assign(l1_s2);
    assign(l2_s1);
    assign(l2_s2);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    std::cout << "Spine-Leaf topology built successfully" << std::endl;

    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
