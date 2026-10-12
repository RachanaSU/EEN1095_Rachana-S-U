// ============================================================
// FACTORY LEAF-SPINE THESIS SIMULATION
// Three-way comparison:
//   1. Packet Spraying  — baseline, high reorder
//   2. PT-ECMP          — train-based, zero reorder, drain gap cost
//   3. Flowlet Switch   — burst-aware, near-zero reorder, no penalty
//
// Topology: Sender → Relay[i] → Receiver
//   Relay[0]: 4us   Relay[1]: 52us
//   Relay[2]: 102us Relay[3]: 152us
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

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("ThesisSimulation");

// ============================================================
// CONFIG
// ============================================================
struct SimConfig {
    uint32_t numPaths        = 4;
    uint32_t totalPackets    = 400;
    uint32_t trainSize       = 16;
    uint32_t packetSize      = 1000;
    double   sendIntervalUs  = 10.0;
    bool     usePTECMP       = false;
    bool     useFlowlet      = false;
    // Flowlet gap: if silence > this, next burst gets a new path
    // Must be > max path delay difference (152-4=148us) → use 200us
    double   flowletGapUs    = 200.0;
    // Burst size: packets per flowlet burst (simulates real traffic bursts)
    uint32_t burstSize       = 16;
    double   burstGapUs      = 250.0;  // gap between bursts (triggers path switch)
    std::string outputPrefix = "results/";
    double pathDelays[4]     = {4.0, 52.0, 102.0, 152.0};
};
static SimConfig g_cfg;

// ============================================================
// PACKET HEADER
// ============================================================
class PktHeader : public Header {
public:
    uint32_t seq=0, trainId=0, pathId=0;
    uint64_t sentNs=0;

    static TypeId GetTypeId() {
        static TypeId tid = TypeId("PktHeader")
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
        os<<"seq="<<seq<<" train="<<trainId<<" path="<<pathId;
    }
};

// ============================================================
// METRICS
// ============================================================
struct Metrics {
    uint32_t sent=0, received=0, reorders=0, maxSeq=0;
    bool first=true;
    uint64_t startNs=0, endNs=0;
    std::vector<uint64_t> delays;
    std::vector<uint32_t> arrivalOrder;
    std::map<uint32_t,uint32_t> pathUsage;

    void Record(uint32_t seq, uint32_t trainId, uint32_t pathId, uint64_t delayNs) {
        delays.push_back(delayNs);
        arrivalOrder.push_back(seq);
        pathUsage[pathId]++;
        bool isReorder = (!first && seq < maxSeq);
        if (isReorder) reorders++;
        if (first || seq > maxSeq) maxSeq = seq;
        first = false;
        received++;

        std::cout << (isReorder ? "!REORDER " : "         ")
                  << "RX seq=" << std::setw(4) << seq
                  << "  train=" << std::setw(3) << trainId
                  << "  path[" << pathId << "]"
                  << "  delay=" << std::fixed << std::setprecision(3)
                  << (delayNs/1e6) << "ms"
                  << (isReorder ? "  <-- OUT OF ORDER!" : "")
                  << "\n";
    }

    double AvgDelayMs() const {
        if(delays.empty()) return 0;
        double s=0; for(auto d:delays) s+=d; return s/delays.size()/1e6;
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

// ============================================================
// RELAY APPLICATION
// Receives packet, schedules forward after path delay.
// This creates REAL discrete-event delay in NS3.
// ============================================================
static Ipv4Address g_receiverAddr;
static uint16_t    g_receiverPort = 9000;

class RelayApp : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("RelayApp")
            .SetParent<Application>().AddConstructor<RelayApp>();
        return tid;
    }
    RelayApp() = default;
    void Setup(uint16_t port, double delayUs) {
        m_port=port; m_delayUs=delayUs;
    }
private:
    void StartApplication() override {
        m_rx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port));
        m_rx->SetRecvCallback(MakeCallback(&RelayApp::OnRecv, this));
        m_tx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_tx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 0));
    }
    void StopApplication() override {
        if(m_rx) m_rx->Close();
        if(m_tx) m_tx->Close();
    }
    void OnRecv(Ptr<Socket> sock) {
        Ptr<Packet> pkt; Address from;
        while((pkt=sock->RecvFrom(from)))
            Simulator::Schedule(MicroSeconds(m_delayUs),
                                &RelayApp::Forward, this, pkt->Copy());
    }
    void Forward(Ptr<Packet> pkt) {
        if(m_tx) m_tx->SendTo(pkt,0,InetSocketAddress(g_receiverAddr,g_receiverPort));
    }
    uint16_t    m_port=0;
    double      m_delayUs=0;
    Ptr<Socket> m_rx, m_tx;
};

// ============================================================
// TOPOLOGY
// ============================================================
static Ptr<Node> g_senderNode;
static Ptr<Node> g_receiverNode;
static std::vector<Ptr<Node>>    g_relayNodes;
static std::vector<Ipv4Address>  g_relayAddrs;

void BuildTopology() {
    g_senderNode   = CreateObject<Node>();
    g_receiverNode = CreateObject<Node>();
    for(uint32_t i=0;i<g_cfg.numPaths;i++)
        g_relayNodes.push_back(CreateObject<Node>());

    NodeContainer all;
    all.Add(g_senderNode); all.Add(g_receiverNode);
    for(auto &r:g_relayNodes) all.Add(r);
    InternetStackHelper inet; inet.Install(all);

    Ipv4AddressHelper ip;
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate",StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay",StringValue("1us"));

    for(uint32_t i=0;i<g_cfg.numPaths;i++) {
        std::ostringstream b1; b1<<"10.1."<<i<<".0";
        ip.SetBase(b1.str().c_str(),"255.255.255.252");
        auto d1=p2p.Install(g_senderNode,g_relayNodes[i]);
        auto i1=ip.Assign(d1);
        g_relayAddrs.push_back(i1.GetAddress(1));
        ip.NewNetwork();

        std::ostringstream b2; b2<<"10.2."<<i<<".0";
        ip.SetBase(b2.str().c_str(),"255.255.255.252");
        auto d2=p2p.Install(g_relayNodes[i],g_receiverNode);
        auto i2=ip.Assign(d2);
        if(i==0) g_receiverAddr=i2.GetAddress(1);
        ip.NewNetwork();
    }
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    std::cout<<"\n╔══════════════════════════════════════════════╗\n";
    std::cout<<"║  TOPOLOGY: Sender → Relay[i] → Receiver     ║\n";
    std::cout<<"╠══════════════════════════════════════════════╣\n";
    for(uint32_t i=0;i<g_cfg.numPaths;i++)
        std::cout<<"║  Path["<<i<<"]: "<<g_cfg.pathDelays[i]
                 <<"us delay  relay="<<g_relayAddrs[i]<<"\n";
    std::cout<<"╚══════════════════════════════════════════════╝\n\n";
}

// ============================================================
// SENDER APPLICATION
// Implements all three modes:
//   Packet Spraying: rotate path every packet
//   PT-ECMP:         block of 64 packets per path + drain gap
//   Flowlet:         burst of burstSize on same path,
//                    then wait burstGapUs before switching path
// ============================================================
class Sender : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("Sender")
            .SetParent<Application>().AddConstructor<Sender>();
        return tid;
    }
    Sender() = default;

private:
    void StartApplication() override {
        m_seq=m_trainId=m_inTrain=0;
        m_currentPath=0;
        m_inBurst=0;

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

        double interval = g_cfg.sendIntervalUs;

        if(g_cfg.usePTECMP) {
            // PT-ECMP: drain gap at each block boundary
            uint32_t blockSize = g_cfg.numPaths * g_cfg.trainSize;
            bool isBlockBoundary = (m_seq>0) && (m_seq % blockSize==0);
            if(isBlockBoundary) interval = 200.0;

        } else if(g_cfg.useFlowlet) {
            // Flowlet: gap between bursts triggers path switch
            bool isBurstBoundary = (m_inBurst >= g_cfg.burstSize);
            if(isBurstBoundary) {
                interval = g_cfg.burstGapUs; // this gap is what flowlet detects
            }
        }

        Simulator::Schedule(MicroSeconds(interval), &Sender::Send, this);
    }

    void Send() {
        if(m_seq>=g_cfg.totalPackets) return;

        uint32_t pathIdx = 0;

        if(g_cfg.usePTECMP) {
            // ── PT-ECMP ────────────────────────────────────────────
            // Block-based: 64 packets (4 trains × 16) per path
            uint32_t blockSize = g_cfg.numPaths * g_cfg.trainSize;
            pathIdx = (m_seq / blockSize) % g_cfg.numPaths;

        } else if(g_cfg.useFlowlet) {
            // ── Flowlet Switching ──────────────────────────────────
            // Within a burst: stay on same path (no reorder possible)
            // At burst boundary: switch to next path
            // The burstGapUs > max path delay difference, so all in-flight
            // packets from the old path arrive before new path starts.
            if(m_inBurst >= g_cfg.burstSize) {
                // Switch path — the gap above ensures drain
                m_currentPath = (m_currentPath + 1) % g_cfg.numPaths;
                m_inBurst = 0;
            }
            pathIdx = m_currentPath;

        } else {
            // ── Packet Spraying ────────────────────────────────────
            // Each packet rotates path → different delays → REORDERS
            pathIdx = m_seq % g_cfg.numPaths;
        }

        auto pkt = Create<Packet>(g_cfg.packetSize);
        PktHeader hdr;
        hdr.seq=m_seq; hdr.trainId=m_trainId;
        hdr.pathId=pathIdx;
        hdr.sentNs=Simulator::Now().GetNanoSeconds();
        pkt->AddHeader(hdr);

        m_sockets[pathIdx]->SendTo(pkt,0,
            InetSocketAddress(g_relayAddrs[pathIdx],7000+pathIdx));

        g_metrics.sent++;
        m_seq++; m_inTrain++; m_inBurst++;
        if(m_inTrain>=g_cfg.trainSize) { m_trainId++; m_inTrain=0; }
        Schedule();
    }

    uint32_t m_seq=0, m_trainId=0, m_inTrain=0;
    uint32_t m_currentPath=0, m_inBurst=0;
    std::vector<Ptr<Socket>> m_sockets;
};

// ============================================================
// RECEIVER APPLICATION
// ============================================================
class Receiver : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("Receiver")
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
            g_metrics.Record(hdr.seq,hdr.trainId,hdr.pathId,now-hdr.sentNs);
        }
    }
    Ptr<Socket> m_sock;
};

// ============================================================
// OUTPUT
// ============================================================
void PrintSummary(const std::string &mode) {
    auto &m=g_metrics;
    std::cout<<"\n╔══════════════════════════════════════════════╗\n";
    std::cout<<"║           SIMULATION RESULTS                 ║\n";
    std::cout<<"╠══════════════════════════════════════════════╣\n";
    std::cout<<"║  Mode       : "<<std::left<<std::setw(31)<<mode        <<"║\n";
    std::cout<<"║  Packets    : "<<std::setw(31)<<m.sent                 <<"║\n";
    std::cout<<"╠══════════════════════════════════════════════╣\n";
    std::cout<<"║  PATH USAGE (load balance)                   ║\n";
    for(uint32_t i=0;i<g_cfg.numPaths;i++) {
        std::ostringstream s;
        s<<"Path["<<i<<"] ("<<g_cfg.pathDelays[i]<<"us): "<<m.pathUsage[i]<<" pkts";
        std::cout<<"║  "<<std::left<<std::setw(44)<<s.str()              <<"║\n";
    }
    std::cout<<"╠══════════════════════════════════════════════╣\n";
    std::cout<<"║  REORDERING                                  ║\n";
    std::cout<<"║  Reorder Events : "<<std::setw(27)<<m.reorders         <<"║\n";
    std::cout<<"║  Reorder Ratio  : "<<std::setw(23)
             <<std::fixed<<std::setprecision(1)<<m.ReorderPct()<<" %    ║\n";
    std::cout<<"╠══════════════════════════════════════════════╣\n";
    std::cout<<"║  LATENCY                                     ║\n";
    std::cout<<"║  Avg Delay  : "<<std::setw(28)
             <<std::fixed<<std::setprecision(4)<<m.AvgDelayMs()<<" ms   ║\n";
    std::cout<<"║  Min Delay  : "<<std::setw(28)<<m.MinDelayMs()         <<" ms   ║\n";
    std::cout<<"║  Max Delay  : "<<std::setw(28)<<m.MaxDelayMs()         <<" ms   ║\n";
    std::cout<<"║  Jitter     : "<<std::setw(28)<<m.JitterMs()           <<" ms   ║\n";
    std::cout<<"╠══════════════════════════════════════════════╣\n";
    std::cout<<"║  THROUGHPUT : "<<std::setw(31)
             <<std::fixed<<std::setprecision(3)<<m.ThroughputMbps()<<" Mbps║\n";
    std::cout<<"╚══════════════════════════════════════════════╝\n";
}

void WriteCSV(const std::string &mode) {
    system("mkdir -p results");
    auto &m=g_metrics;
    std::string fn=g_cfg.outputPrefix+mode+"_results.csv";
    std::ofstream f(fn);
    if(!f.is_open()) { std::cerr<<"Cannot write "<<fn<<"\n"; return; }
    f<<"metric,value\n"
     <<"mode,"<<mode<<"\n"
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
    std::cout<<"CSV   → "<<fn<<"\n";

    std::string fn2=g_cfg.outputPrefix+mode+"_delay_trace.csv";
    std::ofstream f2(fn2);
    f2<<"arrival_order,seq,delay_ms\n";
    for(size_t i=0;i<m.arrivalOrder.size();i++)
        f2<<i<<","<<m.arrivalOrder[i]<<","
          <<std::fixed<<std::setprecision(4)<<(m.delays[i]/1e6)<<"\n";
    f2.close();
    std::cout<<"Trace → "<<fn2<<"\n";
}

// ============================================================
// MAIN
// ============================================================
int main(int argc, char *argv[]) {
    CommandLine cmd;
    cmd.AddValue("ptecmp",      "Enable PT-ECMP mode",        g_cfg.usePTECMP);
    cmd.AddValue("flowlet",     "Enable Flowlet Switching",    g_cfg.useFlowlet);
    cmd.AddValue("trainSize",   "Packets per train (PT-ECMP)", g_cfg.trainSize);
    cmd.AddValue("burstSize",   "Packets per burst (Flowlet)", g_cfg.burstSize);
    cmd.AddValue("burstGap",    "Gap between bursts (us)",     g_cfg.burstGapUs);
    cmd.AddValue("flowletGap",  "Flowlet gap threshold (us)",  g_cfg.flowletGapUs);
    cmd.AddValue("packets",     "Total packets",               g_cfg.totalPackets);
    cmd.AddValue("sendInterval","Send interval (us)",          g_cfg.sendIntervalUs);
    cmd.AddValue("output",      "Output prefix",               g_cfg.outputPrefix);
    cmd.Parse(argc, argv);

    std::string mode = g_cfg.useFlowlet  ? "Flowlet-Switching"
                     : g_cfg.usePTECMP   ? "PT-ECMP"
                                         : "Packet-Spraying";

    std::cout<<"\n╔══════════════════════════════════════════════╗\n";
    std::cout<<"║  FACTORY LEAF-SPINE THESIS SIMULATION        ║\n";
    std::cout<<"║  Mode    : "<<std::left<<std::setw(34)<<mode           <<"║\n";
    std::cout<<"║  Packets : "<<std::setw(34)<<g_cfg.totalPackets        <<"║\n";
    if(g_cfg.useFlowlet)
        std::cout<<"║  Burst   : "<<std::setw(34)<<g_cfg.burstSize       <<"║\n";
    else
        std::cout<<"║  Train   : "<<std::setw(34)<<g_cfg.trainSize       <<"║\n";
    std::cout<<"╚══════════════════════════════════════════════╝\n";

    BuildTopology();

    // Install relay apps
    for(uint32_t i=0;i<g_cfg.numPaths;i++) {
        auto relay=CreateObject<RelayApp>();
        relay->Setup(7000+i, g_cfg.pathDelays[i]);
        g_relayNodes[i]->AddApplication(relay);
        relay->SetStartTime(Seconds(0.1));
        relay->SetStopTime(Seconds(300.0));
    }

    auto sender=CreateObject<Sender>();
    g_senderNode->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0));
    sender->SetStopTime(Seconds(300.0));

    auto receiver=CreateObject<Receiver>();
    g_receiverNode->AddApplication(receiver);
    receiver->SetStartTime(Seconds(0.5));
    receiver->SetStopTime(Seconds(300.0));

    Simulator::Stop(Seconds(305.0));
    Simulator::Run();
    if(g_metrics.endNs==0) g_metrics.endNs=Simulator::Now().GetNanoSeconds();

    PrintSummary(mode);
    WriteCSV(mode);
    Simulator::Destroy();
    return 0;
}
