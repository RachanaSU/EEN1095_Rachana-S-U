#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/socket.h"

using namespace ns3;

class SeqTag : public Tag
{
public:
    uint32_t seq;

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("SeqTag").SetParent<Tag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 4; }
    void Serialize(TagBuffer i) const override { i.WriteU32(seq); }
    void Deserialize(TagBuffer i) override { seq = i.ReadU32(); }
    void Print(std::ostream &os) const override { os << seq; }
};

// ---------------- GLOBALS ----------------
static uint32_t g_seq = 0;
static uint32_t g_last = 0;
static uint32_t g_reorder = 0;

// ---------------- RECEIVE ----------------
static void ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;

    while ((packet = socket->RecvFrom(from)))
    {
        SeqTag tag;
        packet->PeekPacketTag(tag);

        std::cout << "RX seq=" << tag.seq << std::endl;

        if (tag.seq < g_last)
        {
            g_reorder++;
            std::cout << ">>> REORDER <<<" << std::endl;
        }

        g_last = tag.seq;
    }
}

// ---------------- SEND ----------------
static void Send(Ptr<Socket> socket, Ipv4Address dst, bool fast)
{
    Ptr<Packet> p = Create<Packet>(512);

    SeqTag tag;
    tag.seq = g_seq++;
    p->AddPacketTag(tag);

    socket->SendTo(p, 0, InetSocketAddress(dst, 9));

    std::cout << "TX seq=" << tag.seq
              << (fast ? " via FAST" : " via SLOW") << std::endl;
}

int main()
{
    std::cout << "=== TRUE PACKET SPRAY (FIXED ECMP MODEL) ===" << std::endl;

    NodeContainer nodes;
    nodes.Create(2);

    InternetStackHelper internet;
    internet.InstallAll();

    // ---------------- TWO PATHS ----------------
    PointToPointHelper fast, slow;

    fast.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    fast.SetChannelAttribute("Delay", StringValue("2ms"));

    slow.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    slow.SetChannelAttribute("Delay", StringValue("25ms"));

    NetDeviceContainer d1 = fast.Install(nodes.Get(0), nodes.Get(1));
    NetDeviceContainer d2 = slow.Install(nodes.Get(0), nodes.Get(1));

    Ipv4AddressHelper address;

    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i1 = address.Assign(d1);

    address.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer i2 = address.Assign(d2);

    Ptr<Node> sender = nodes.Get(0);
    Ptr<Node> receiver = nodes.Get(1);

    Ptr<Socket> fastSock = Socket::CreateSocket(sender, TypeId::LookupByName("ns3::UdpSocketFactory"));
    Ptr<Socket> slowSock = Socket::CreateSocket(sender, TypeId::LookupByName("ns3::UdpSocketFactory"));

    Ptr<Socket> rx = Socket::CreateSocket(receiver, TypeId::LookupByName("ns3::UdpSocketFactory"));

    rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9));
    rx->SetRecvCallback(MakeCallback(&ReceivePacket));

    fastSock->Bind();
    slowSock->Bind();

    Ipv4Address dstFast = i1.GetAddress(1);
    Ipv4Address dstSlow = i2.GetAddress(1);

    // ---------------- TRUE SPRAY ----------------
    for (int i = 0; i < 100; i++)
    {
        Simulator::Schedule(Seconds(1.0 + i * 0.0005), [=]()
        {
            if (i % 2 == 0)
                Send(fastSock, dstFast, true);
            else
                Send(slowSock, dstSlow, false);
        });
    }

    Simulator::Stop(Seconds(5));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "FINAL REORDER COUNT = " << g_reorder << std::endl;

    return 0;
}
