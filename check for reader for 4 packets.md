#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/tag.h"
#include "ns3/socket.h"

using namespace ns3;

// ---------------- PACKET TAG (for sequence numbers) ----------------
class SeqTag : public Tag
{
public:
    uint32_t seq;

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("SeqTag")
            .SetParent<Tag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 4; }

    void Serialize(TagBuffer i) const override { i.WriteU32(seq); }
    void Deserialize(TagBuffer i) override { seq = i.ReadU32(); }

    void Print(std::ostream &os) const override
    {
        os << "seq=" << seq;
    }
};

static uint32_t g_seq = 0;
static uint32_t g_lastSeq = 0;
static uint32_t g_reorder = 0;

// ---------------- SEND ----------------
static void SendPacket(Ptr<Socket> socket, Ipv4Address dst, uint16_t port)
{
    Ptr<Packet> packet = Create<Packet>(512);

    SeqTag tag;
    tag.seq = g_seq++;
    packet->AddPacketTag(tag);

    socket->SendTo(packet, 0, InetSocketAddress(dst, port));

    std::cout << "Sent packet seq=" << tag.seq << std::endl;
}

// ---------------- RECEIVE (FIXED CALLBACK) ----------------
static void ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;

    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        SeqTag tag;
        if (packet->PeekPacketTag(tag))
        {
            std::cout << "Received seq=" << tag.seq << std::endl;

            if (tag.seq < g_lastSeq)
            {
                g_reorder++;
                std::cout << ">>> REORDER DETECTED <<<" << std::endl;
            }

            g_lastSeq = tag.seq;
        }
    }
}

// ---------------- MAIN ----------------
int main()
{
    std::cout << "Packet Spraying + Reordering FIXED Simulation" << std::endl;

    NodeContainer hosts, leaves, spines;
    hosts.Create(2);
    leaves.Create(2);
    spines.Create(2);

    InternetStackHelper internet;
    internet.InstallAll();

    PointToPointHelper fast, slow;

    fast.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    fast.SetChannelAttribute("Delay", StringValue("2ms"));

    slow.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    slow.SetChannelAttribute("Delay", StringValue("10ms"));

    // topology
    fast.Install(hosts.Get(0), leaves.Get(0));
    fast.Install(hosts.Get(1), leaves.Get(1));

    NetDeviceContainer l0s0 = fast.Install(leaves.Get(0), spines.Get(0));
    NetDeviceContainer l0s1 = slow.Install(leaves.Get(0), spines.Get(1));
    NetDeviceContainer l1s0 = fast.Install(leaves.Get(1), spines.Get(0));
    NetDeviceContainer l1s1 = slow.Install(leaves.Get(1), spines.Get(1));

    Ipv4AddressHelper address;
    int subnet = 1;

    auto assign = [&](NetDeviceContainer d)
    {
        std::ostringstream base;
        base << "10.1." << subnet++ << ".0";
        address.SetBase(base.str().c_str(), "255.255.255.0");
        return address.Assign(d);
    };

    auto i1 = assign(l0s0);
    auto i2 = assign(l0s1);
    auto i3 = assign(l1s0);
    auto i4 = assign(l1s1);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    Ptr<Node> sender = hosts.Get(0);
    Ptr<Node> receiver = hosts.Get(1);

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    Ptr<Socket> s1 = Socket::CreateSocket(sender, tid);
    Ptr<Socket> s2 = Socket::CreateSocket(sender, tid);

    Ptr<Socket> recv = Socket::CreateSocket(receiver, tid);
    recv->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9));
    recv->SetRecvCallback(MakeCallback(&ReceivePacket));

    s1->Bind();
    s2->Bind();

    Ipv4Address dst = i3.GetAddress(0);

    Simulator::Schedule(Seconds(2.0), &SendPacket, s1, dst, 9);
    Simulator::Schedule(Seconds(2.1), &SendPacket, s2, dst, 9);
    Simulator::Schedule(Seconds(2.2), &SendPacket, s1, dst, 9);
    Simulator::Schedule(Seconds(2.3), &SendPacket, s2, dst, 9);

    Simulator::Stop(Seconds(5.0));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "Reorder count = " << g_reorder << std::endl;

    return 0;
}
