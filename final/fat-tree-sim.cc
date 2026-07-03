// ============================================================
// FAT-TREE THESIS SIMULATION
// k=4 Fat-Tree: 4 pods, 4 core switches, 8 aggregation, 8 edge
// Three-way comparison:
//   1. Packet Spraying  — baseline, high reorder
//   2. PT-ECMP          — train-based, zero reorder, drain gap
//   3. Flowlet Switching — burst-aware, zero reorder, no penalty
//
// Traffic path: Host(sender) → Edge → Aggr → Core[i] → Aggr → Edge → Host(receiver)
// 4 parallel paths through Core[0..3], each with different delay
// Core[0]=4us Core[1]=52us Core[2]=102us Core[3]=152us
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
NS_LOG_COMPONENT_DEFINE("FatTreeSimulation");

// ============================================================
// CONFIG
// ============================================================
struct SimConfig {
    uint32_t k             = 4;       // fat-tree parameter
    uint32_t numPaths      = 4;       // = k (core switches = k paths)
    uint32_t totalPackets  = 400;
    uint32_t trainSize     = 16;
    uint32_t burstSize     = 16;
    uint32_t packetSize    = 1000;
    double   sendIntervalUs = 10.0;
    double   burstGapUs    = 250.0;
    bool     usePTECMP     = false;
    bool     useFlowlet    = false;
    std::string outputPrefix = "results/fattree_";
    // Core switch forwarding delays (model asymmetric paths)
    double   coreDelays[4] = {4.0, 52.0, 102.0, 152.0};
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
        static TypeId tid = TypeId("FTPktHeader")
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

    void Record(uint32_t seq, uint32_t trainId,
                uint32_t pathId, uint64_t delayNs) {
        delays.push_back(delayNs);
        arrivalOrder.push_back(seq);
        pathUsage[pathId]++;
        bool isReorder = (!first && seq < maxSeq);
        if(isReorder) reorders++;
        if(first || seq > maxSeq) maxSeq = seq;
        first = false;
        received++;

        std::cout << (isReorder ? "!REORDER " : "         ")
                  << "RX seq=" << std::setw(4) << seq
                  << "  train=" << std::setw(3) << trainId
                  << "  core[" << pathId << "]"
                  << "  delay=" << std::fixed << std::setprecision(3)
                  << (delayNs/1e6) << "ms"
                  << (isReorder ? "  <-- OUT OF ORDER!" : "")
                  << "\n";
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

// ============================================================
// CORE RELAY APPLICATION
// Models fat-tree core switch forwarding delay
// Packet path: sender -> edge -> aggr -> CoreRelay -> aggr -> edge -> receiver
// The CoreRelay introduces the asymmetric per-path delay
// ============================================================
static Ipv4Address g_receiverAddr;
static uint16_t    g_receiverPort = 9000;

class CoreRelay : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("CoreRelay")
            .SetParent<Application>().AddConstructor<CoreRelay>();
        return tid;
    }
    CoreRelay() = default;
    void Setup(uint16_t port, double delayUs, uint32_t coreId) {
        m_port=port; m_delayUs=delayUs; m_coreId=coreId;
    }
private:
    void StartApplication() override {
        m_rx=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
        m_rx->Bind(InetSocketAddress(Ipv4Address::GetAny(),m_port));
        m_rx->SetRecvCallback(MakeCallback(&CoreRelay::OnRecv,this));
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
                &CoreRelay::Forward,this,pkt->Copy());
    }
    void Forward(Ptr<Packet> pkt) {
        if(m_tx)
            m_tx->SendTo(pkt,0,
                InetSocketAddress(g_receiverAddr,g_receiverPort));
    }
    uint16_t    m_port=0;
    double      m_delayUs=0;
    uint32_t    m_coreId=0;
    Ptr<Socket> m_rx,m_tx;
};

// ============================================================
// FAT-TREE TOPOLOGY
// k=4: 4 pods, 4 core switches, 8 aggr, 8 edge, 2 hosts
// For simulation purposes we model 1 sender host and 1 receiver
// host, connected through the full fat-tree switch hierarchy,
// with traffic reaching the receiver via one of 4 core switches.
// ============================================================
static Ptr<Node> g_senderNode;
static Ptr<Node> g_receiverNode;
static std::vector<Ptr<Node>> g_coreNodes;      // 4 core switches
static std::vector<Ptr<Node>> g_aggrNodes;      // 8 aggr switches
static std::vector<Ptr<Node>> g_edgeNodes;      // 8 edge switches
static std::vector<Ipv4Address> g_coreAddrs;    // sender-to-core addresses

void BuildFatTree() {
    uint32_t k = g_cfg.k; // k=4

    // Create nodes
    g_senderNode   = CreateObject<Node>();
    g_receiverNode = CreateObject<Node>();

    // k core switches
    for(uint32_t i=0;i<k;i++)
        g_coreNodes.push_back(CreateObject<Node>());

    // k pods * k/2 aggr per pod = k*k/2 aggr switches
    for(uint32_t i=0;i<k*k/2;i++)
        g_aggrNodes.push_back(CreateObject<Node>());

    // k pods * k/2 edge per pod = k*k/2 edge switches
    for(uint32_t i=0;i<k*k/2;i++)
        g_edgeNodes.push_back(CreateObject<Node>());

    // Install internet stack on all
    NodeContainer all;
    all.Add(g_senderNode);
    all.Add(g_receiverNode);
    for(auto &n:g_coreNodes)  all.Add(n);
    for(auto &n:g_aggrNodes)  all.Add(n);
    for(auto &n:g_edgeNodes)  all.Add(n);
    InternetStackHelper inet;
    inet.Install(all);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate",StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay",StringValue("1us"));
    Ipv4AddressHelper ip;

    uint32_t subnet=0;
    auto nextSubnet = [&]() -> std::string {
        std::ostringstream s;
        s<<"10."<<(subnet/256)<<"."<<(subnet%256)<<".0";
        subnet++;
        return s.str();
    };

    // ── Sender → sender-side edge switch (pod 0, edge 0) ──
    ip.SetBase(nextSubnet().c_str(),"255.255.255.252");
    auto d=p2p.Install(g_senderNode,g_edgeNodes[0]);
    ip.Assign(d);
    ip.NewNetwork();

    // ── Sender-side edge → sender-side aggr switches ──
    // In k=4 fat-tree, each edge connects to k/2=2 aggr in same pod
    for(uint32_t a=0;a<k/2;a++) {
        ip.SetBase(nextSubnet().c_str(),"255.255.255.252");
        auto d2=p2p.Install(g_edgeNodes[0],g_aggrNodes[a]);
        ip.Assign(d2);
        ip.NewNetwork();
    }

    // ── Sender-side aggr → core switches ──
    // Each aggr in pod connects to k/2 core switches
    // Aggr 0 → Core 0,1  |  Aggr 1 → Core 2,3
    for(uint32_t a=0;a<k/2;a++) {
        for(uint32_t c=a*(k/2);c<(a+1)*(k/2);c++) {
            ip.SetBase(nextSubnet().c_str(),"255.255.255.252");
            auto d3=p2p.Install(g_aggrNodes[a],g_coreNodes[c]);
            auto i3=ip.Assign(d3);
            g_coreAddrs.push_back(i3.GetAddress(1)); // core IP
            ip.NewNetwork();
        }
    }

    // ── Core → receiver-side aggr switches ──
    // Core 0,1 → Aggr 4 (pod 1)  |  Core 2,3 → Aggr 5 (pod 1)
    for(uint32_t c=0;c<k;c++) {
        uint32_t aggrIdx = 2 + (c/(k/2));  // aggr 2 or 3
        ip.SetBase(nextSubnet().c_str(),"255.255.255.252");
        auto d4=p2p.Install(g_coreNodes[c],g_aggrNodes[aggrIdx]);
        ip.Assign(d4);
        ip.NewNetwork();
    }

    // ── Receiver-side aggr → receiver-side edge ──
    for(uint32_t a=2;a<4;a++) {
        ip.SetBase(nextSubnet().c_str(),"255.255.255.252");
        auto d5=p2p.Install(g_aggrNodes[a],g_edgeNodes[1]);
        ip.Assign(d5);
        ip.NewNetwork();
    }

    // ── Receiver-side edge → receiver host ──
    ip.SetBase(nextSubnet().c_str(),"255.255.255.252");
    auto d6=p2p.Install(g_edgeNodes[1],g_receiverNode);
    auto i6=ip.Assign(d6);
    g_receiverAddr=i6.GetAddress(1);
    ip.NewNetwork();

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Print topology summary
    std::cout<<"\n╔══════════════════════════════════════════════════╗\n";
    std::cout<<"║  k="<<k<<" FAT-TREE TOPOLOGY                          ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  "<<k<<" Core switches (asymmetric delays)            ║\n";
    std::cout<<"║  "<<k*k/2<<" Aggregation switches                       ║\n";
    std::cout<<"║  "<<k*k/2<<" Edge switches                              ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  Traffic path:                                   ║\n";
    std::cout<<"║  Sender→Edge→Aggr→Core[i]→Aggr→Edge→Receiver   ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    for(uint32_t i=0;i<g_cfg.numPaths;i++)
        std::cout<<"║  Core["<<i<<"]: delay="<<g_cfg.coreDelays[i]
                 <<"us  addr="<<g_coreAddrs[i]<<"\n";
    std::cout<<"║  Receiver: "<<g_receiverAddr<<"\n";
    std::cout<<"╚══════════════════════════════════════════════════╝\n\n";
}

// ============================================================
// SENDER APPLICATION
// Same three routing modes as spine-leaf sim
// ============================================================
class FTSender : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("FTSender")
            .SetParent<Application>().AddConstructor<FTSender>();
        return tid;
    }
    FTSender() = default;

private:
    void StartApplication() override {
        m_seq=m_trainId=m_inTrain=m_inBurst=m_currentPath=0;
        for(uint32_t i=0;i<g_cfg.numPaths;i++) {
            auto s=Socket::CreateSocket(GetNode(),
                        UdpSocketFactory::GetTypeId());
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
            if(m_seq>0 && m_seq%blockSize==0) interval=200.0;
        } else if(g_cfg.useFlowlet) {
            if(m_inBurst>=g_cfg.burstSize) interval=g_cfg.burstGapUs;
        }
        Simulator::Schedule(MicroSeconds(interval),&FTSender::Send,this);
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

        // Send to core switch for this path
        m_sockets[pathIdx]->SendTo(pkt,0,
            InetSocketAddress(g_coreAddrs[pathIdx],7000+pathIdx));

        g_metrics.sent++;
        m_seq++; m_inTrain++; m_inBurst++;
        if(m_inTrain>=g_cfg.trainSize){m_trainId++;m_inTrain=0;}
        Schedule();
    }

    uint32_t m_seq=0,m_trainId=0,m_inTrain=0;
    uint32_t m_currentPath=0,m_inBurst=0;
    std::vector<Ptr<Socket>> m_sockets;
};

// ============================================================
// RECEIVER APPLICATION
// ============================================================
class FTReceiver : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("FTReceiver")
            .SetParent<Application>().AddConstructor<FTReceiver>();
        return tid;
    }
    FTReceiver() = default;
private:
    void StartApplication() override {
        m_sock=Socket::CreateSocket(GetNode(),
                    UdpSocketFactory::GetTypeId());
        m_sock->Bind(InetSocketAddress(Ipv4Address::GetAny(),
                    g_receiverPort));
        m_sock->SetRecvCallback(
            MakeCallback(&FTReceiver::OnRecv,this));
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
            g_metrics.Record(hdr.seq,hdr.trainId,
                             hdr.pathId,now-hdr.sentNs);
        }
    }
    Ptr<Socket> m_sock;
};

// ============================================================
// OUTPUT
// ============================================================
void PrintSummary(const std::string &mode) {
    auto &m=g_metrics;
    std::cout<<"\n╔══════════════════════════════════════════════════╗\n";
    std::cout<<"║           FAT-TREE SIMULATION RESULTS            ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  Topology   : k=4 Fat-Tree                       ║\n";
    std::cout<<"║  Mode       : "<<std::left<<std::setw(35)<<mode<<"║\n";
    std::cout<<"║  Packets    : "<<std::setw(35)<<m.sent<<"║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  PATH USAGE via Core switches                    ║\n";
    for(uint32_t i=0;i<g_cfg.numPaths;i++) {
        std::ostringstream s;
        s<<"Core["<<i<<"] ("<<g_cfg.coreDelays[i]
         <<"us): "<<m.pathUsage[i]<<" pkts";
        std::cout<<"║  "<<std::left<<std::setw(48)<<s.str()<<"║\n";
    }
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  REORDERING                                      ║\n";
    std::cout<<"║  Reorder Events : "<<std::setw(31)<<m.reorders<<"║\n";
    std::cout<<"║  Reorder Ratio  : "<<std::setw(27)
             <<std::fixed<<std::setprecision(1)
             <<m.ReorderPct()<<" %    ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  LATENCY                                         ║\n";
    std::cout<<"║  Avg Delay  : "<<std::setw(30)
             <<std::fixed<<std::setprecision(4)
             <<m.AvgDelayMs()<<" ms   ║\n";
    std::cout<<"║  Min Delay  : "<<std::setw(30)<<m.MinDelayMs()<<" ms   ║\n";
    std::cout<<"║  Max Delay  : "<<std::setw(30)<<m.MaxDelayMs()<<" ms   ║\n";
    std::cout<<"║  Jitter     : "<<std::setw(30)<<m.JitterMs()<<" ms   ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  THROUGHPUT : "<<std::setw(31)
             <<std::fixed<<std::setprecision(3)
             <<m.ThroughputMbps()<<" Mbps║\n";
    std::cout<<"╚══════════════════════════════════════════════════╝\n";
}

void WriteCSV(const std::string &mode) {
    system("mkdir -p results");
    auto &m=g_metrics;
    std::string fn=g_cfg.outputPrefix+mode+"_results.csv";
    std::ofstream f(fn);
    f<<"metric,value\n"
     <<"topology,k4-fat-tree\n"
     <<"mode,"<<mode<<"\n"
     <<"sent,"<<m.sent<<"\n"
     <<"received,"<<m.received<<"\n"
     <<"reorder_events,"<<m.reorders<<"\n"
     <<"reorder_pct,"<<std::fixed<<std::setprecision(2)
     <<m.ReorderPct()<<"\n"
     <<"avg_delay_ms,"<<m.AvgDelayMs()<<"\n"
     <<"min_delay_ms,"<<m.MinDelayMs()<<"\n"
     <<"max_delay_ms,"<<m.MaxDelayMs()<<"\n"
     <<"jitter_ms,"<<m.JitterMs()<<"\n"
     <<"throughput_mbps,"<<m.ThroughputMbps()<<"\n";
    for(uint32_t i=0;i<g_cfg.numPaths;i++)
        f<<"core"<<i<<"_pkts,"<<m.pathUsage[i]<<"\n";
    f.close();
    std::cout<<"CSV   → "<<fn<<"\n";

    std::string fn2=g_cfg.outputPrefix+mode+"_delay_trace.csv";
    std::ofstream f2(fn2);
    f2<<"arrival_order,seq,delay_ms\n";
    for(size_t i=0;i<m.arrivalOrder.size();i++)
        f2<<i<<","<<m.arrivalOrder[i]<<","
          <<std::fixed<<std::setprecision(4)
          <<(m.delays[i]/1e6)<<"\n";
    f2.close();
    std::cout<<"Trace → "<<fn2<<"\n";
}

// ============================================================
// MAIN
// ============================================================
int main(int argc, char *argv[]) {
    CommandLine cmd;
    cmd.AddValue("ptecmp",   "Enable PT-ECMP mode",       g_cfg.usePTECMP);
    cmd.AddValue("flowlet",  "Enable Flowlet Switching",   g_cfg.useFlowlet);
    cmd.AddValue("trainSize","Packets per train",          g_cfg.trainSize);
    cmd.AddValue("burstSize","Packets per burst",          g_cfg.burstSize);
    cmd.AddValue("packets",  "Total packets",              g_cfg.totalPackets);
    cmd.AddValue("output",   "Output prefix",              g_cfg.outputPrefix);
    cmd.Parse(argc,argv);

    std::string mode = g_cfg.useFlowlet  ? "Flowlet-Switching"
                     : g_cfg.usePTECMP   ? "PT-ECMP"
                                         : "Packet-Spraying";

    std::cout<<"\n╔══════════════════════════════════════════════════╗\n";
    std::cout<<"║  k=4 FAT-TREE THESIS SIMULATION                  ║\n";
    std::cout<<"║  Mode    : "<<std::left<<std::setw(38)<<mode<<"║\n";
    std::cout<<"║  Packets : "<<std::setw(38)<<g_cfg.totalPackets<<"║\n";
    std::cout<<"╚══════════════════════════════════════════════════╝\n";

    BuildFatTree();

    // Install core relay apps (model core switch forwarding delay)
    for(uint32_t i=0;i<g_cfg.numPaths;i++) {
        auto relay=CreateObject<CoreRelay>();
        relay->Setup(7000+i,g_cfg.coreDelays[i],i);
        g_coreNodes[i]->AddApplication(relay);
        relay->SetStartTime(Seconds(0.1));
        relay->SetStopTime(Seconds(300.0));
    }

    auto sender=CreateObject<FTSender>();
    g_senderNode->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0));
    sender->SetStopTime(Seconds(300.0));

    auto receiver=CreateObject<FTReceiver>();
    g_receiverNode->AddApplication(receiver);
    receiver->SetStartTime(Seconds(0.5));
    receiver->SetStopTime(Seconds(300.0));

    Simulator::Stop(Seconds(305.0));
    Simulator::Run();
    if(g_metrics.endNs==0)
        g_metrics.endNs=Simulator::Now().GetNanoSeconds();

    PrintSummary(mode);
    WriteCSV(mode);
    Simulator::Destroy();
    return 0;
}