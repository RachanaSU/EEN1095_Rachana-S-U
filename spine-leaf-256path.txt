// ============================================================
// 256-PATH SPINE-LEAF SIMULATION
// Extended from 4-path baseline to 256 parallel paths
// Path delays: 4us to 514us (2us increment, 256 paths)
// Max delay difference: 510us
// Burst gap (flowlet): 610us
// Drain gap (PT-ECMP): 610us
//
// Three algorithms compared:
//   1. Packet Spraying   — baseline
//   2. PT-ECMP           — block=4096 pkts + 610us drain gap
//   3. Flowlet Switching — burst=16 pkts + 610us gap
// ============================================================

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"

#include <fstream>
#include <iomanip>
#include <vector>
#include <map>
#include <algorithm>
#include <sstream>
#include <cmath>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("Sim256Path");

// ── CONFIG ────────────────────────────────────────────────
struct SimConfig {
    uint32_t numPaths       = 256;
    uint32_t totalPackets   = 16384;
    uint32_t trainSize      = 16;
    uint32_t burstSize      = 16;
    uint32_t packetSize     = 1000;
    double   sendIntervalUs = 10.0;
    double   burstGapUs     = 610.0;
    bool     usePTECMP      = false;
    bool     useFlowlet     = false;
    std::string outputPrefix = "results/p256_";
    double pathDelays[256] = {4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220, 222, 224, 226, 228, 230, 232, 234, 236, 238, 240, 242, 244, 246, 248, 250, 252, 254, 256, 258, 260, 262, 264, 266, 268, 270, 272, 274, 276, 278, 280, 282, 284, 286, 288, 290, 292, 294, 296, 298, 300, 302, 304, 306, 308, 310, 312, 314, 316, 318, 320, 322, 324, 326, 328, 330, 332, 334, 336, 338, 340, 342, 344, 346, 348, 350, 352, 354, 356, 358, 360, 362, 364, 366, 368, 370, 372, 374, 376, 378, 380, 382, 384, 386, 388, 390, 392, 394, 396, 398, 400, 402, 404, 406, 408, 410, 412, 414, 416, 418, 420, 422, 424, 426, 428, 430, 432, 434, 436, 438, 440, 442, 444, 446, 448, 450, 452, 454, 456, 458, 460, 462, 464, 466, 468, 470, 472, 474, 476, 478, 480, 482, 484, 486, 488, 490, 492, 494, 496, 498, 500, 502, 504, 506, 508, 510, 512, 514};
};
static SimConfig g_cfg;

// ── PACKET HEADER ─────────────────────────────────────────
class PktHeader : public Header {
public:
    uint32_t seq=0, trainId=0, pathId=0;
    uint64_t sentNs=0;
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("P256Hdr")
            .SetParent<Header>().AddConstructor<PktHeader>();
        return tid;
    }
    TypeId   GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize()  const override { return 20; }
    void Serialize(Buffer::Iterator i) const override {
        i.WriteHtonU32(seq); i.WriteHtonU32(trainId);
        i.WriteHtonU32(pathId); i.WriteHtonU64(sentNs);
    }
    uint32_t Deserialize(Buffer::Iterator i) override {
        seq=i.ReadNtohU32(); trainId=i.ReadNtohU32();
        pathId=i.ReadNtohU32(); sentNs=i.ReadNtohU64();
        return 20;
    }
    void Print(std::ostream &os) const override {
        os<<"seq="<<seq<<" path="<<pathId;
    }
};

// ── METRICS ───────────────────────────────────────────────
struct Metrics {
    uint32_t sent=0, received=0, reorders=0, maxSeq=0;
    bool first=true;
    uint64_t startNs=0, endNs=0;
    std::vector<uint64_t> delays;
    std::map<uint32_t,uint32_t> pathUsage;

    void Record(uint32_t seq, uint32_t pathId, uint64_t delayNs) {
        delays.push_back(delayNs);
        pathUsage[pathId]++;
        bool isReorder = (!first && seq < maxSeq);
        if(isReorder) reorders++;
        if(first || seq > maxSeq) maxSeq = seq;
        first = false; received++;
        // Only print first 20 and summary markers to avoid flooding
        if(received <= 20 || received % 1000 == 0)
            std::cout << (isReorder?"!REORDER ":"         ")
                      << "RX seq=" << std::setw(5) << seq
                      << "  path[" << std::setw(3) << pathId << "]"
                      << "  delay=" << std::fixed << std::setprecision(3)
                      << (delayNs/1e6) << "ms"
                      << (isReorder?" <-- OUT OF ORDER!":"") << "\n";
    }
    double AvgDelayMs() const {
        if(delays.empty()) return 0;
        double s=0; for(auto d:delays) s+=d;
        return s/delays.size()/1e6;
    }
    double MinDelayMs() const {
        if(delays.empty()) return 0;
        return *std::min_element(delays.begin(),delays.end())/1e6;
    }
    double MaxDelayMs() const {
        if(delays.empty()) return 0;
        return *std::max_element(delays.begin(),delays.end())/1e6;
    }
    double JitterMs() const {
        if(delays.size()<2) return 0;
        double s=0;
        for(size_t i=1;i<delays.size();i++)
            s+=std::abs((int64_t)delays[i]-(int64_t)delays[i-1]);
        return s/(delays.size()-1)/1e6;
    }
    double ReorderPct() const {
        return received?(double)reorders/received*100.0:0;
    }
    double ThroughputMbps() const {
        if(endNs<=startNs||received==0) return 0;
        return (double)received*g_cfg.packetSize*8.0/(endNs-startNs)*1e3;
    }
};
static Metrics g_metrics;

// ── RELAY APP ─────────────────────────────────────────────
static Ipv4Address g_receiverAddr;
static uint16_t    g_receiverPort = 9000;

class RelayApp : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("P256Relay")
            .SetParent<Application>().AddConstructor<RelayApp>();
        return tid;
    }
    RelayApp() = default;
    void Setup(uint16_t port, double delayUs) {
        m_port=port; m_delayUs=delayUs;
    }
private:
    void StartApplication() override {
        m_rx=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
        m_rx->Bind(InetSocketAddress(Ipv4Address::GetAny(),m_port));
        m_rx->SetRecvCallback(MakeCallback(&RelayApp::OnRecv,this));
        m_tx=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
        m_tx->Bind(InetSocketAddress(Ipv4Address::GetAny(),0));
    }
    void StopApplication() override {
        if(m_rx) m_rx->Close();
        if(m_tx) m_tx->Close();
    }
    void OnRecv(Ptr<Socket> sock) {
        Ptr<Packet> pkt; Address from;
        while((pkt=sock->RecvFrom(from)))
            Simulator::Schedule(MicroSeconds(m_delayUs),
                &RelayApp::Forward,this,pkt->Copy());
    }
    void Forward(Ptr<Packet> pkt) {
        if(m_tx)
            m_tx->SendTo(pkt,0,
                InetSocketAddress(g_receiverAddr,g_receiverPort));
    }
    uint16_t    m_port=0;
    double      m_delayUs=0;
    Ptr<Socket> m_rx,m_tx;
};

// ── TOPOLOGY ──────────────────────────────────────────────
static Ptr<Node> g_senderNode;
static Ptr<Node> g_receiverNode;
static std::vector<Ptr<Node>>   g_relayNodes;
static std::vector<Ipv4Address> g_relayAddrs;

void BuildTopology() {
    g_senderNode   = CreateObject<Node>();
    g_receiverNode = CreateObject<Node>();
    for(uint32_t i=0;i<g_cfg.numPaths;i++)
        g_relayNodes.push_back(CreateObject<Node>());

    NodeContainer all;
    all.Add(g_senderNode); all.Add(g_receiverNode);
    for(auto &r:g_relayNodes) all.Add(r);
    InternetStackHelper inet; inet.Install(all);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate",StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay",StringValue("1us"));
    Ipv4AddressHelper ip;

    uint32_t subnet=0;
    for(uint32_t i=0;i<g_cfg.numPaths;i++) {
        std::ostringstream b1;
        b1<<"10."<<(subnet/256)<<"."<<(subnet%256)<<".0";
        ip.SetBase(b1.str().c_str(),"255.255.255.252");
        auto d1=p2p.Install(g_senderNode,g_relayNodes[i]);
        auto i1=ip.Assign(d1);
        g_relayAddrs.push_back(i1.GetAddress(1));
        ip.NewNetwork(); subnet++;

        std::ostringstream b2;
        b2<<"10."<<(subnet/256)<<"."<<(subnet%256)<<".0";
        ip.SetBase(b2.str().c_str(),"255.255.255.252");
        auto d2=p2p.Install(g_relayNodes[i],g_receiverNode);
        auto i2=ip.Assign(d2);
        if(i==0) g_receiverAddr=i2.GetAddress(1);
        ip.NewNetwork(); subnet++;
    }
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    std::cout<<"\n╔══════════════════════════════════════════════════╗\n";
    std::cout<<"║  256-PATH SPINE-LEAF TOPOLOGY                    ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  Paths     : 256                                   ║\n";
    std::cout<<"║  Min delay : 4us                                    ║\n";
    std::cout<<"║  Max delay : 514us                                  ║\n";
    std::cout<<"║  Max diff  : 510us                                  ║\n";
    std::cout<<"╚══════════════════════════════════════════════════╝\n\n";
}

// ── SENDER ────────────────────────────────────────────────
class Sender : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("P256Sender")
            .SetParent<Application>().AddConstructor<Sender>();
        return tid;
    }
    Sender() = default;
private:
    void StartApplication() override {
        m_seq=m_trainId=m_inTrain=m_inBurst=m_currentPath=0;
        for(uint32_t i=0;i<g_cfg.numPaths;i++) {
            auto s=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
            s->Bind(InetSocketAddress(Ipv4Address::GetAny(),6000+i));
            m_sockets.push_back(s);
        }
        g_metrics.startNs=Simulator::Now().GetNanoSeconds();
        Schedule();
    }
    void StopApplication() override {
        for(auto &s:m_sockets) if(s) s->Close();
        m_sockets.clear();
    }
    void Schedule() {
        if(m_seq>=g_cfg.totalPackets) return;
        double interval=g_cfg.sendIntervalUs;
        if(g_cfg.usePTECMP) {
            uint32_t blockSize=g_cfg.numPaths*g_cfg.trainSize;
            if(m_seq>0 && m_seq%blockSize==0) interval=610.0;
        } else if(g_cfg.useFlowlet) {
            if(m_inBurst>=g_cfg.burstSize) interval=g_cfg.burstGapUs;
        }
        Simulator::Schedule(MicroSeconds(interval),&Sender::Send,this);
    }
    void Send() {
        if(m_seq>=g_cfg.totalPackets) return;
        uint32_t pathIdx=0;
        if(g_cfg.usePTECMP) {
            uint32_t blockSize=g_cfg.numPaths*g_cfg.trainSize;
            pathIdx=(m_seq/blockSize)%g_cfg.numPaths;
        } else if(g_cfg.useFlowlet) {
            if(m_inBurst>=g_cfg.burstSize) {
                m_currentPath=(m_currentPath+1)%g_cfg.numPaths;
                m_inBurst=0;
            }
            pathIdx=m_currentPath;
        } else {
            pathIdx=m_seq%g_cfg.numPaths;
        }
        auto pkt=Create<Packet>(g_cfg.packetSize);
        PktHeader hdr;
        hdr.seq=m_seq; hdr.trainId=m_trainId;
        hdr.pathId=pathIdx;
        hdr.sentNs=Simulator::Now().GetNanoSeconds();
        pkt->AddHeader(hdr);
        m_sockets[pathIdx]->SendTo(pkt,0,
            InetSocketAddress(g_relayAddrs[pathIdx],7000+pathIdx));
        g_metrics.sent++;
        m_seq++; m_inTrain++; m_inBurst++;
        if(m_inTrain>=g_cfg.trainSize){m_trainId++;m_inTrain=0;}
        Schedule();
    }
    uint32_t m_seq=0,m_trainId=0,m_inTrain=0;
    uint32_t m_currentPath=0,m_inBurst=0;
    std::vector<Ptr<Socket>> m_sockets;
};

// ── RECEIVER ──────────────────────────────────────────────
class Receiver : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("P256Receiver")
            .SetParent<Application>().AddConstructor<Receiver>();
        return tid;
    }
    Receiver() = default;
private:
    void StartApplication() override {
        m_sock=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
        m_sock->Bind(InetSocketAddress(Ipv4Address::GetAny(),g_receiverPort));
        m_sock->SetRecvCallback(MakeCallback(&Receiver::OnRecv,this));
    }
    void StopApplication() override {
        if(m_sock) m_sock->Close();
        g_metrics.endNs=Simulator::Now().GetNanoSeconds();
    }
    void OnRecv(Ptr<Socket> sock) {
        Ptr<Packet> pkt; Address from;
        while((pkt=sock->RecvFrom(from))) {
            PktHeader hdr; pkt->RemoveHeader(hdr);
            uint64_t now=Simulator::Now().GetNanoSeconds();
            g_metrics.Record(hdr.seq,hdr.pathId,now-hdr.sentNs);
        }
    }
    Ptr<Socket> m_sock;
};

// ── OUTPUT ────────────────────────────────────────────────
void PrintSummary(const std::string &mode) {
    auto &m=g_metrics;
    std::cout<<"\n╔══════════════════════════════════════════════════╗\n";
    std::cout<<"║       256-PATH SIMULATION RESULTS                ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  Mode       : "<<std::left<<std::setw(35)<<mode<<"║\n";
    std::cout<<"║  Paths      : "<<std::setw(35)<<g_cfg.numPaths<<"║\n";
    std::cout<<"║  Sent       : "<<std::setw(35)<<m.sent<<"║\n";
    std::cout<<"║  Received   : "<<std::setw(35)<<m.received<<"║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  PATH USAGE (sample — first 8 paths)             ║\n";
    for(uint32_t i=0;i<8;i++) {
        std::ostringstream s;
        s<<"Path["<<i<<"] ("<<g_cfg.pathDelays[i]<<"us): "<<m.pathUsage[i]<<" pkts";
        std::cout<<"║  "<<std::left<<std::setw(48)<<s.str()<<"║\n";
    }
    std::cout<<"║  ...                                             ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  REORDERING                                      ║\n";
    std::cout<<"║  Reorder Events : "<<std::setw(31)<<m.reorders<<"║\n";
    std::cout<<"║  Reorder Ratio  : "<<std::setw(27)
             <<std::fixed<<std::setprecision(2)<<m.ReorderPct()<<" %    ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  LATENCY                                         ║\n";
    std::cout<<"║  Avg Delay  : "<<std::setw(30)
             <<std::fixed<<std::setprecision(4)<<m.AvgDelayMs()<<" ms   ║\n";
    std::cout<<"║  Min Delay  : "<<std::setw(30)<<m.MinDelayMs()<<" ms   ║\n";
    std::cout<<"║  Max Delay  : "<<std::setw(30)<<m.MaxDelayMs()<<" ms   ║\n";
    std::cout<<"║  Jitter     : "<<std::setw(30)<<m.JitterMs()<<" ms   ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  THROUGHPUT : "<<std::setw(31)
             <<std::fixed<<std::setprecision(3)<<m.ThroughputMbps()<<" Mbps║\n";
    std::cout<<"╚══════════════════════════════════════════════════╝\n";
}

void WriteCSV(const std::string &mode) {
    system("mkdir -p results");
    auto &m=g_metrics;
    std::string fn=g_cfg.outputPrefix+mode+"_results.csv";
    std::ofstream f(fn);
    f<<"metric,value\n"
     <<"mode,"<<mode<<"\n"
     <<"num_paths,256\n"
     <<"sent,"<<m.sent<<"\n"
     <<"received,"<<m.received<<"\n"
     <<"reorder_events,"<<m.reorders<<"\n"
     <<"reorder_pct,"<<std::fixed<<std::setprecision(2)<<m.ReorderPct()<<"\n"
     <<"avg_delay_ms,"<<m.AvgDelayMs()<<"\n"
     <<"min_delay_ms,"<<m.MinDelayMs()<<"\n"
     <<"max_delay_ms,"<<m.MaxDelayMs()<<"\n"
     <<"jitter_ms,"<<m.JitterMs()<<"\n"
     <<"throughput_mbps,"<<m.ThroughputMbps()<<"\n";
    for(uint32_t i=0;i<g_cfg.numPaths;i++)
        f<<"path"<<i<<"_pkts,"<<m.pathUsage[i]<<"\n";
    f.close();
    std::cout<<"CSV → "<<fn<<"\n";

    std::string fn2=g_cfg.outputPrefix+mode+"_delay_trace.csv";
    std::ofstream f2(fn2);
    f2<<"arrival_order,seq,delay_ms\n";
    for(size_t i=0;i<m.delays.size();i++)
        f2<<i<<","<<i<<","
          <<std::fixed<<std::setprecision(4)<<(m.delays[i]/1e6)<<"\n";
    f2.close();
    std::cout<<"Trace → "<<fn2<<"\n";
}

// ── MAIN ──────────────────────────────────────────────────
int main(int argc, char *argv[]) {
    CommandLine cmd;
    cmd.AddValue("ptecmp",   "PT-ECMP mode",         g_cfg.usePTECMP);
    cmd.AddValue("flowlet",  "Flowlet mode",          g_cfg.useFlowlet);
    cmd.AddValue("packets",  "Total packets",         g_cfg.totalPackets);
    cmd.AddValue("output",   "Output prefix",         g_cfg.outputPrefix);
    cmd.Parse(argc,argv);

    std::string mode = g_cfg.useFlowlet ? "Flowlet-Switching"
                     : g_cfg.usePTECMP  ? "PT-ECMP"
                                        : "Packet-Spraying";

    std::cout<<"\n╔══════════════════════════════════════════════════╗\n";
    std::cout<<"║  256-PATH SPINE-LEAF THESIS SIMULATION           ║\n";
    std::cout<<"║  Mode    : "<<std::left<<std::setw(38)<<mode<<"║\n";
    std::cout<<"║  Packets : "<<std::setw(38)<<g_cfg.totalPackets<<"║\n";
    std::cout<<"║  Paths   : "<<std::setw(38)<<g_cfg.numPaths<<"║\n";
    std::cout<<"╚══════════════════════════════════════════════════╝\n";

    BuildTopology();

    for(uint32_t i=0;i<g_cfg.numPaths;i++) {
        auto relay=CreateObject<RelayApp>();
        relay->Setup(7000+i,g_cfg.pathDelays[i]);
        g_relayNodes[i]->AddApplication(relay);
        relay->SetStartTime(Seconds(0.1));
        relay->SetStopTime(Seconds(3600.0));
    }

    auto sender=CreateObject<Sender>();
    g_senderNode->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0));
    sender->SetStopTime(Seconds(3600.0));

    auto receiver=CreateObject<Receiver>();
    g_receiverNode->AddApplication(receiver);
    receiver->SetStartTime(Seconds(0.5));
    receiver->SetStopTime(Seconds(3600.0));

    Simulator::Stop(Seconds(3605.0));
    Simulator::Run();
    if(g_metrics.endNs==0)
        g_metrics.endNs=Simulator::Now().GetNanoSeconds();

    PrintSummary(mode);
    WriteCSV(mode);
    Simulator::Destroy();
    return 0;
}
