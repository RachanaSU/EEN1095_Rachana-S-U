// ============================================================
// ADVANCED SPINE-LEAF SIMULATION — Publication Extension
// Extends the basic thesis simulation with:
//   1. Bursty Pareto on-off traffic (realistic traffic model)
//   2. Multiple senders and receivers (4 each)
//   3. LetFlow as a fourth algorithm
//
// Four algorithms compared:
//   1. Packet Spraying   — baseline, high reorder
//   2. PT-ECMP           — train-based, zero reorder, drain gap
//   3. Flowlet Switching — burst-aware, zero reorder, no penalty
//   4. LetFlow           — adaptive flowlet, picks least loaded path
//
// Topology:
//   Senders[0..3] → Relay/Spine[0..3] → Receivers[0..3]
//   Each sender can reach each receiver via 4 parallel paths
//   Path delays: 4us, 52us, 102us, 152us
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
#include <numeric>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("AdvancedSpineLeaf");

// ============================================================
// CONFIG
// ============================================================
struct SimConfig {
    uint32_t numPaths       = 4;
    uint32_t numSenders     = 4;   // multiple senders
    uint32_t numReceivers   = 4;   // multiple receivers
    uint32_t totalPackets   = 800; // more packets for larger scale
    uint32_t trainSize      = 16;
    uint32_t burstSize      = 16;
    uint32_t packetSize     = 1000;
    double   sendIntervalUs = 10.0;
    double   burstGapUs     = 250.0;
    // Pareto on-off traffic model parameters
    // ON period: Pareto distributed (models bursty real traffic)
    // OFF period: Pareto distributed (models idle gaps)
    double   onMeanUs       = 160.0;  // mean ON burst duration
    double   offMeanUs      = 250.0;  // mean OFF idle duration
    double   paretoShape    = 1.5;    // Pareto shape (1.5 = heavy tail)
    bool     usePTECMP      = false;
    bool     useFlowlet     = false;
    bool     useLetFlow     = false;
    // LetFlow parameters
    // LetFlow picks the path with the lowest estimated queue length
    // It tracks per-path packet counts as a proxy for load
    double   letFlowGapUs   = 250.0; // gap threshold same as flowlet
    double   pathDelays[4]  = {4.0, 52.0, 102.0, 152.0};
    std::string outputPrefix = "results/advanced_";
};
static SimConfig g_cfg;

// ============================================================
// PACKET HEADER
// ============================================================
class PktHeader : public Header {
public:
    uint32_t seq=0, trainId=0, pathId=0, senderId=0;
    uint64_t sentNs=0;

    static TypeId GetTypeId() {
        static TypeId tid = TypeId("AdvPktHeader")
            .SetParent<Header>().AddConstructor<PktHeader>();
        return tid;
    }
    TypeId   GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize()  const override { return 24; }
    void Serialize(Buffer::Iterator i) const override {
        i.WriteHtonU32(seq); i.WriteHtonU32(trainId);
        i.WriteHtonU32(pathId); i.WriteHtonU32(senderId);
        i.WriteHtonU64(sentNs);
    }
    uint32_t Deserialize(Buffer::Iterator i) override {
        seq=i.ReadNtohU32(); trainId=i.ReadNtohU32();
        pathId=i.ReadNtohU32(); senderId=i.ReadNtohU32();
        sentNs=i.ReadNtohU64();
        return 24;
    }
    void Print(std::ostream &os) const override {
        os<<"seq="<<seq<<" path="<<pathId<<" sender="<<senderId;
    }
};

// ============================================================
// METRICS — per sender
// ============================================================
struct Metrics {
    uint32_t sent=0, received=0, reorders=0, maxSeq=0;
    bool first=true;
    uint64_t startNs=0, endNs=0;
    std::vector<uint64_t> delays;
    std::vector<uint32_t> arrivalOrder;
    std::map<uint32_t,uint32_t> pathUsage;

    void Record(uint32_t seq, uint32_t pathId, uint64_t delayNs) {
        delays.push_back(delayNs);
        arrivalOrder.push_back(seq);
        pathUsage[pathId]++;
        bool isReorder = (!first && seq < maxSeq);
        if(isReorder) reorders++;
        if(first || seq > maxSeq) maxSeq = seq;
        first = false;
        received++;
    }
    double AvgDelayMs() const {
        if(delays.empty()) return 0;
        double s=0; for(auto d:delays) s+=d;
        return s/delays.size()/1e6;
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

// Global metrics — one per sender
static std::vector<Metrics> g_metrics;
static std::vector<Ipv4Address> g_receiverAddrs;
static uint16_t g_receiverPort = 9000;

// ============================================================
// RELAY APPLICATION
// ============================================================
class RelayApp : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("AdvRelayApp")
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
        while((pkt=sock->RecvFrom(from))) {
            // Read sender ID from header to route to correct receiver
            PktHeader hdr; pkt->PeekHeader(hdr);
            uint32_t receiverId = hdr.senderId % g_cfg.numReceivers;
            Simulator::Schedule(MicroSeconds(m_delayUs),
                &RelayApp::Forward,this,pkt->Copy(),receiverId);
        }
    }
    void Forward(Ptr<Packet> pkt, uint32_t receiverId) {
        if(m_tx && receiverId < g_receiverAddrs.size())
            m_tx->SendTo(pkt,0,
                InetSocketAddress(g_receiverAddrs[receiverId],
                                  g_receiverPort));
    }
    uint16_t    m_port=0;
    double      m_delayUs=0;
    Ptr<Socket> m_rx,m_tx;
};

// ============================================================
// TOPOLOGY
// ============================================================
static std::vector<Ptr<Node>> g_senderNodes;
static std::vector<Ptr<Node>> g_receiverNodes;
static std::vector<Ptr<Node>> g_relayNodes;
static std::vector<Ipv4Address> g_relayAddrs;

void BuildTopology() {
    // Create nodes
    for(uint32_t i=0;i<g_cfg.numSenders;i++)
        g_senderNodes.push_back(CreateObject<Node>());
    for(uint32_t i=0;i<g_cfg.numReceivers;i++)
        g_receiverNodes.push_back(CreateObject<Node>());
    for(uint32_t i=0;i<g_cfg.numPaths;i++)
        g_relayNodes.push_back(CreateObject<Node>());

    NodeContainer all;
    for(auto &n:g_senderNodes)   all.Add(n);
    for(auto &n:g_receiverNodes) all.Add(n);
    for(auto &n:g_relayNodes)    all.Add(n);
    InternetStackHelper inet; inet.Install(all);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate",StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay",StringValue("1us"));
    Ipv4AddressHelper ip;
    uint32_t subnet=0;
    auto nextNet = [&]() {
        std::ostringstream s;
        s<<"10."<<subnet/256<<"."<<subnet%256<<".0";
        subnet++; return s.str();
    };

    // Connect each sender to each relay
    for(uint32_t s=0;s<g_cfg.numSenders;s++) {
        for(uint32_t r=0;r<g_cfg.numPaths;r++) {
            ip.SetBase(nextNet().c_str(),"255.255.255.252");
            auto d=p2p.Install(g_senderNodes[s],g_relayNodes[r]);
            auto i=ip.Assign(d);
            // Store relay addr from first sender's perspective
            if(s==0) g_relayAddrs.push_back(i.GetAddress(1));
            ip.NewNetwork();
        }
    }

    // Connect each relay to each receiver
    for(uint32_t r=0;r<g_cfg.numPaths;r++) {
        for(uint32_t recv=0;recv<g_cfg.numReceivers;recv++) {
            ip.SetBase(nextNet().c_str(),"255.255.255.252");
            auto d=p2p.Install(g_relayNodes[r],g_receiverNodes[recv]);
            auto i=ip.Assign(d);
            if(r==0) g_receiverAddrs.push_back(i.GetAddress(1));
            ip.NewNetwork();
        }
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    std::cout<<"\n╔══════════════════════════════════════════════════╗\n";
    std::cout<<"║  ADVANCED SPINE-LEAF TOPOLOGY                    ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  Senders   : "<<std::setw(35)<<g_cfg.numSenders<<"║\n";
    std::cout<<"║  Receivers : "<<std::setw(35)<<g_cfg.numReceivers<<"║\n";
    std::cout<<"║  Paths     : "<<std::setw(35)<<g_cfg.numPaths<<"║\n";
    std::cout<<"║  Traffic   : Pareto on-off bursty model          ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    for(uint32_t i=0;i<g_cfg.numPaths;i++)
        std::cout<<"║  Path["<<i<<"]: "<<g_cfg.pathDelays[i]
                 <<"us  relay="<<g_relayAddrs[i]<<"\n";
    std::cout<<"╚══════════════════════════════════════════════════╝\n\n";
}

// ============================================================
// PARETO ON-OFF TRAFFIC GENERATOR
// Models real bursty data centre traffic:
//   ON  period: sends packets continuously (Pareto distributed)
//   OFF period: silent (Pareto distributed)
// Pareto distribution produces heavy-tailed behaviour —
// most bursts are short but occasionally very long,
// matching real measured data centre traffic patterns
// (Benson et al. 2010, Chowdhury et al. 2012)
// ============================================================
class ParetoSender : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ParetoSender")
            .SetParent<Application>().AddConstructor<ParetoSender>();
        return tid;
    }
    ParetoSender() = default;
    void Setup(uint32_t senderId) { m_senderId = senderId; }

private:
    // Generate Pareto-distributed random value
    // Using inverse transform sampling:
    // X = scale / U^(1/shape) where U ~ Uniform(0,1)
    double ParetoDraw(double mean, double shape) {
        double scale = mean * (shape - 1.0) / shape;
        double u = m_rng->GetValue();
        if(u <= 0) u = 1e-9;
        return scale / std::pow(u, 1.0/shape);
    }

    void StartApplication() override {
        m_seq=m_trainId=m_inTrain=m_inBurst=m_currentPath=0;
        m_isOn = true;
        m_pathLoad.assign(g_cfg.numPaths, 0);

        // Seed random number generator differently per sender
        m_rng = CreateObject<UniformRandomVariable>();
        m_rng->SetAttribute("Min", DoubleValue(0.0));
        m_rng->SetAttribute("Max", DoubleValue(1.0));

        for(uint32_t i=0;i<g_cfg.numPaths;i++) {
            auto s=Socket::CreateSocket(GetNode(),
                        UdpSocketFactory::GetTypeId());
            s->Bind(InetSocketAddress(Ipv4Address::GetAny(),
                        6000+m_senderId*10+i));
            m_sockets.push_back(s);
        }

        if(m_senderId < g_metrics.size())
            g_metrics[m_senderId].startNs =
                Simulator::Now().GetNanoSeconds();

        // Start in ON state
        ScheduleOnPeriod();
    }

    void StopApplication() override {
        for(auto &s:m_sockets) if(s) s->Close();
        m_sockets.clear();
        m_sendEvent.Cancel();
    }

    void ScheduleOnPeriod() {
        // Draw ON duration from Pareto distribution
        double onDuration = ParetoDraw(g_cfg.onMeanUs,
                                       g_cfg.paretoShape);
        m_onEndUs = Simulator::Now().GetMicroSeconds() + onDuration;
        m_isOn = true;
        SendPacket();
    }

    void ScheduleOffPeriod() {
        // Draw OFF duration from Pareto distribution
        double offDuration = ParetoDraw(g_cfg.offMeanUs,
                                        g_cfg.paretoShape);
        m_isOn = false;
        // After OFF period, start new ON period
        Simulator::Schedule(MicroSeconds(offDuration),
                            &ParetoSender::ScheduleOnPeriod, this);
    }

    void SendPacket() {
        if(m_seq >= g_cfg.totalPackets) return;
        if(!m_isOn) return;

        // Check if ON period has expired
        if(Simulator::Now().GetMicroSeconds() > m_onEndUs) {
            ScheduleOffPeriod();
            return;
        }

        uint32_t pathIdx = SelectPath();
        SendOnPath(pathIdx);

        // Schedule next packet
        double interval = g_cfg.sendIntervalUs;
        if(g_cfg.useFlowlet || g_cfg.useLetFlow) {
            if(m_inBurst >= g_cfg.burstSize)
                interval = g_cfg.burstGapUs;
        } else if(g_cfg.usePTECMP) {
            uint32_t blockSize = g_cfg.numPaths * g_cfg.trainSize;
            if(m_seq > 0 && m_seq % blockSize == 0)
                interval = 200.0;
        }

        m_sendEvent = Simulator::Schedule(
            MicroSeconds(interval),
            &ParetoSender::SendPacket, this);
    }

    uint32_t SelectPath() {
        if(g_cfg.usePTECMP) {
            uint32_t blockSize = g_cfg.numPaths * g_cfg.trainSize;
            return (m_seq / blockSize) % g_cfg.numPaths;

        } else if(g_cfg.useFlowlet) {
            if(m_inBurst >= g_cfg.burstSize) {
                m_currentPath = (m_currentPath+1) % g_cfg.numPaths;
                m_inBurst = 0;
            }
            return m_currentPath;

        } else if(g_cfg.useLetFlow) {
            // LetFlow: at burst boundary, pick LEAST LOADED path
            // Load = cumulative packets sent on each path this session
            // This ensures even distribution across all paths over time
            // while keeping each burst on a single path (no reordering)
            if(m_inBurst >= g_cfg.burstSize) {
                uint32_t minLoad = UINT32_MAX;
                uint32_t bestPath = 0;
                // Find path with fewest total packets sent so far
                // Tie-break by path index to avoid always picking 0
                for(uint32_t i=0;i<g_cfg.numPaths;i++) {
                    if(m_pathLoad[i] < minLoad ||
                      (m_pathLoad[i] == minLoad && i != m_currentPath)) {
                        minLoad = m_pathLoad[i];
                        bestPath = i;
                    }
                }
                // Don't reuse same path if others are available
                if(bestPath == m_currentPath && g_cfg.numPaths > 1) {
                    for(uint32_t i=0;i<g_cfg.numPaths;i++) {
                        if(i != m_currentPath && m_pathLoad[i] <= minLoad) {
                            bestPath = i; break;
                        }
                    }
                }
                m_currentPath = bestPath;
                m_inBurst = 0;
            }
            return m_currentPath;

        } else {
            // Packet spraying: rotate every packet
            return m_seq % g_cfg.numPaths;
        }
    }

    void SendOnPath(uint32_t pathIdx) {
        auto pkt = Create<Packet>(g_cfg.packetSize);
        PktHeader hdr;
        hdr.seq=m_seq; hdr.trainId=m_trainId;
        hdr.pathId=pathIdx; hdr.senderId=m_senderId;
        hdr.sentNs=Simulator::Now().GetNanoSeconds();
        pkt->AddHeader(hdr);

        m_sockets[pathIdx]->SendTo(pkt,0,
            InetSocketAddress(g_relayAddrs[pathIdx],
                              7000+pathIdx));

        if(m_senderId < g_metrics.size())
            g_metrics[m_senderId].sent++;

        // Load tracking: increment cumulative counter per path
        // For LetFlow: never drains — tracks total packets sent per path
        // so the least-used path is always chosen at burst boundaries
        m_pathLoad[pathIdx]++;

        m_seq++; m_inTrain++; m_inBurst++;
        if(m_inTrain >= g_cfg.trainSize) {
            m_trainId++; m_inTrain=0;
        }
    }

    void DrainPath(uint32_t pathIdx) {
        if(m_pathLoad[pathIdx] > 0)
            m_pathLoad[pathIdx]--;
    }

    uint32_t m_senderId=0;
    uint32_t m_seq=0, m_trainId=0, m_inTrain=0;
    uint32_t m_inBurst=0, m_currentPath=0;
    bool m_isOn=true;
    double m_onEndUs=0;
    std::vector<uint32_t> m_pathLoad;
    std::vector<Ptr<Socket>> m_sockets;
    Ptr<UniformRandomVariable> m_rng;
    EventId m_sendEvent;
};

// ============================================================
// RECEIVER APPLICATION
// ============================================================
class AdvReceiver : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("AdvReceiver")
            .SetParent<Application>().AddConstructor<AdvReceiver>();
        return tid;
    }
    AdvReceiver() = default;
    void Setup(uint32_t receiverId) { m_receiverId=receiverId; }
private:
    void StartApplication() override {
        m_sock=Socket::CreateSocket(GetNode(),
                    UdpSocketFactory::GetTypeId());
        m_sock->Bind(InetSocketAddress(Ipv4Address::GetAny(),
                    g_receiverPort));
        m_sock->SetRecvCallback(
            MakeCallback(&AdvReceiver::OnRecv,this));
    }
    void StopApplication() override {
        if(m_sock) m_sock->Close();
        for(auto &m:g_metrics)
            if(m.endNs==0)
                m.endNs=Simulator::Now().GetNanoSeconds();
    }
    void OnRecv(Ptr<Socket> sock) {
        Ptr<Packet> pkt; Address from;
        while((pkt=sock->RecvFrom(from))) {
            PktHeader hdr; pkt->RemoveHeader(hdr);
            uint64_t now=Simulator::Now().GetNanoSeconds();
            if(hdr.senderId < g_metrics.size()) {
                g_metrics[hdr.senderId].Record(
                    hdr.seq, hdr.pathId, now-hdr.sentNs);
            }
        }
    }
    uint32_t    m_receiverId=0;
    Ptr<Socket> m_sock;
};

// ============================================================
// OUTPUT
// ============================================================
void PrintSummary(const std::string &mode) {
    // Aggregate metrics across all senders
    uint32_t totalSent=0, totalReceived=0, totalReorders=0;
    double totalDelay=0, totalJitter=0;
    std::map<uint32_t,uint32_t> totalPathUsage;

    for(auto &m:g_metrics) {
        totalSent     += m.sent;
        totalReceived += m.received;
        totalReorders += m.reorders;
        totalDelay    += m.AvgDelayMs();
        totalJitter   += m.JitterMs();
        for(auto &p:m.pathUsage) totalPathUsage[p.first]+=p.second;
    }

    double avgDelay  = totalDelay  / g_metrics.size();
    double avgJitter = totalJitter / g_metrics.size();
    double reorderPct = totalReceived ?
        (double)totalReorders/totalReceived*100.0 : 0;

    std::cout<<"\n╔══════════════════════════════════════════════════╗\n";
    std::cout<<"║       ADVANCED SIMULATION RESULTS                ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  Mode       : "<<std::left<<std::setw(35)<<mode<<"║\n";
    std::cout<<"║  Traffic    : Pareto On-Off Bursty               ║\n";
    std::cout<<"║  Senders    : "<<std::setw(35)<<g_cfg.numSenders<<"║\n";
    std::cout<<"║  Receivers  : "<<std::setw(35)<<g_cfg.numReceivers<<"║\n";
    std::cout<<"║  Total Sent : "<<std::setw(35)<<totalSent<<"║\n";
    std::cout<<"║  Total Recv : "<<std::setw(35)<<totalReceived<<"║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  PATH USAGE (aggregated across all senders)      ║\n";
    for(uint32_t i=0;i<g_cfg.numPaths;i++) {
        std::ostringstream s;
        s<<"Path["<<i<<"] ("<<g_cfg.pathDelays[i]
         <<"us): "<<totalPathUsage[i]<<" pkts";
        std::cout<<"║  "<<std::left<<std::setw(48)<<s.str()<<"║\n";
    }
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  REORDERING                                      ║\n";
    std::cout<<"║  Reorder Events : "<<std::setw(31)<<totalReorders<<"║\n";
    std::cout<<"║  Reorder Ratio  : "<<std::setw(27)
             <<std::fixed<<std::setprecision(1)
             <<reorderPct<<" %    ║\n";
    std::cout<<"╠══════════════════════════════════════════════════╣\n";
    std::cout<<"║  LATENCY (averaged across senders)               ║\n";
    std::cout<<"║  Avg Delay  : "<<std::setw(30)
             <<std::fixed<<std::setprecision(4)
             <<avgDelay<<" ms   ║\n";
    std::cout<<"║  Avg Jitter : "<<std::setw(30)
             <<avgJitter<<" ms   ║\n";
    std::cout<<"╚══════════════════════════════════════════════════╝\n";
}

void WriteCSV(const std::string &mode) {
    system("mkdir -p results");
    uint32_t totalSent=0,totalReceived=0,totalReorders=0;
    double totalDelay=0,totalJitter=0;
    std::map<uint32_t,uint32_t> totalPathUsage;
    for(auto &m:g_metrics) {
        totalSent+=m.sent; totalReceived+=m.received;
        totalReorders+=m.reorders;
        totalDelay+=m.AvgDelayMs(); totalJitter+=m.JitterMs();
        for(auto &p:m.pathUsage) totalPathUsage[p.first]+=p.second;
    }
    double reorderPct=totalReceived?
        (double)totalReorders/totalReceived*100.0:0;

    std::string fn=g_cfg.outputPrefix+mode+"_results.csv";
    std::ofstream f(fn);
    f<<"metric,value\n"
     <<"mode,"<<mode<<"\n"
     <<"traffic_model,Pareto-OnOff\n"
     <<"num_senders,"<<g_cfg.numSenders<<"\n"
     <<"num_receivers,"<<g_cfg.numReceivers<<"\n"
     <<"total_sent,"<<totalSent<<"\n"
     <<"total_received,"<<totalReceived<<"\n"
     <<"reorder_events,"<<totalReorders<<"\n"
     <<"reorder_pct,"<<std::fixed<<std::setprecision(2)
     <<reorderPct<<"\n"
     <<"avg_delay_ms,"<<totalDelay/g_metrics.size()<<"\n"
     <<"avg_jitter_ms,"<<totalJitter/g_metrics.size()<<"\n";
    for(uint32_t i=0;i<g_cfg.numPaths;i++)
        f<<"path"<<i<<"_pkts,"<<totalPathUsage[i]<<"\n";
    f.close();
    std::cout<<"CSV → "<<fn<<"\n";
}

// ============================================================
// MAIN
// ============================================================
int main(int argc, char *argv[]) {
    CommandLine cmd;
    cmd.AddValue("ptecmp",    "PT-ECMP mode",           g_cfg.usePTECMP);
    cmd.AddValue("flowlet",   "Flowlet mode",            g_cfg.useFlowlet);
    cmd.AddValue("letflow",   "LetFlow mode",            g_cfg.useLetFlow);
    cmd.AddValue("packets",   "Packets per sender",      g_cfg.totalPackets);
    cmd.AddValue("senders",   "Number of senders",       g_cfg.numSenders);
    cmd.AddValue("receivers", "Number of receivers",     g_cfg.numReceivers);
    cmd.AddValue("onMean",    "Mean ON duration (us)",   g_cfg.onMeanUs);
    cmd.AddValue("offMean",   "Mean OFF duration (us)",  g_cfg.offMeanUs);
    cmd.AddValue("output",    "Output prefix",           g_cfg.outputPrefix);
    cmd.Parse(argc,argv);

    std::string mode = g_cfg.useLetFlow  ? "LetFlow"
                     : g_cfg.useFlowlet  ? "Flowlet-Switching"
                     : g_cfg.usePTECMP   ? "PT-ECMP"
                                         : "Packet-Spraying";

    std::cout<<"\n╔══════════════════════════════════════════════════╗\n";
    std::cout<<"║  ADVANCED SPINE-LEAF SIMULATION                  ║\n";
    std::cout<<"║  Mode    : "<<std::left<<std::setw(38)<<mode<<"║\n";
    std::cout<<"║  Traffic : Pareto On-Off Bursty Model            ║\n";
    std::cout<<"║  Scale   : "<<g_cfg.numSenders<<" senders, "
             <<g_cfg.numReceivers<<" receivers               ║\n";
    std::cout<<"╚══════════════════════════════════════════════════╝\n";

    // Initialise metrics for each sender
    g_metrics.resize(g_cfg.numSenders);

    BuildTopology();

    // Install relay apps
    for(uint32_t i=0;i<g_cfg.numPaths;i++) {
        auto relay=CreateObject<RelayApp>();
        relay->Setup(7000+i, g_cfg.pathDelays[i]);
        g_relayNodes[i]->AddApplication(relay);
        relay->SetStartTime(Seconds(0.1));
        relay->SetStopTime(Seconds(300.0));
    }

    // Install sender apps
    for(uint32_t i=0;i<g_cfg.numSenders;i++) {
        auto sender=CreateObject<ParetoSender>();
        sender->Setup(i);
        g_senderNodes[i]->AddApplication(sender);
        // Stagger sender start times slightly to avoid synchronisation
        sender->SetStartTime(Seconds(1.0 + i*0.01));
        sender->SetStopTime(Seconds(300.0));
    }

    // Install receiver apps
    for(uint32_t i=0;i<g_cfg.numReceivers;i++) {
        auto receiver=CreateObject<AdvReceiver>();
        receiver->Setup(i);
        g_receiverNodes[i]->AddApplication(receiver);
        receiver->SetStartTime(Seconds(0.5));
        receiver->SetStopTime(Seconds(300.0));
    }

    Simulator::Stop(Seconds(305.0));
    Simulator::Run();

    for(auto &m:g_metrics)
        if(m.endNs==0)
            m.endNs=Simulator::Now().GetNanoSeconds();

    PrintSummary(mode);
    WriteCSV(mode);
    Simulator::Destroy();
    return 0;
}