/*
 * hybrid-mpc-dumbbell-dynamic.cc  —  Hybrid MPC+PID+RLS Dynamic Model
 * =====================================================================
 * Flow composition (exact — identical to PHAQM for fair comparison):
 *
 *  Base  (200): 120 BulkSend TCP + 60 OnOff TCP + 20 UDP CBR
 *  Extra  (50):  30 BulkSend TCP + 15 OnOff TCP +  5 UDP CBR  (per pool)
 *
 *  Pool layout:
 *    Extra-A (200-249): 30 BulkTCP + 15 OnOffTCP + 5 UDP  [start t=25s]
 *    Extra-B (250-299): 30 BulkTCP + 15 OnOffTCP + 5 UDP  [t=50s, stop t=85s]
 *    Extra-C (300-349): 30 BulkTCP + 15 OnOffTCP + 5 UDP  [start t=110s]
 *
 *  Total nodes: 350
 *
 * Dynamic event schedule:
 *   t= 0s  : 200 flows, qRef=500
 *   t=25s  : +50 (30B+15W+5U) => 250 total
 *   t=40s  : qRef -> 300
 *   t=50s  : +50 (30B+15W+5U) => 300 total
 *   t=60s  : -60 (first 60 BulkTCP base stop) => 240 total
 *   t=75s  : qRef -> 100  (stress test)
 *   t=85s  : -50 (Extra-B stops) => 190 total
 *   t=95s  : qRef -> 700
 *   t=110s : +50 (30B+15W+5U) => 240 total
 *
 * Controller gains (optimised for fast re-settling):
 *   Kp=0.35  Ki=0.40  Kd=0.012  Horizon=25
 *
 * Run 5 seeds:
 *   for seed in 1 2 3 4 5; do
 *     ./ns3 run "scratch/hybrid-mpc-dumbbell-dynamic --seed=$seed \
 *       --outDir=$HOME/results_dynamic/mpc/run_$seed"
 *   done
 * =====================================================================
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/mpc-queue-disc.h"

#include <iomanip>
#include <sys/stat.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("HybridMpcDumbbellDynamic");

static std::ofstream     g_qLog, g_eventLog, g_phaseLog;
static Ptr<MPCQueueDisc> g_qdisc;
static uint32_t          g_currentQ    = 0;
static double            g_phaseQRef   = 500.0, g_phaseStart = 0.0;
static double            g_phaseQSum   = 0, g_phaseQSumSq = 0;
static uint32_t          g_phaseSamples = 0;
static int               g_phaseNum    = 1;

static void QueueTrace (uint32_t, uint32_t v)
{
  g_currentQ = v;
  g_qLog << Simulator::Now ().GetSeconds () << "," << v << "\n";
  if (Simulator::Now ().GetSeconds () - g_phaseStart > 8.0) {
    g_phaseQSum += v; g_phaseQSumSq += (double)v * v; g_phaseSamples++;
  }
}

static void FlushPhase (const char* reason)
{
  if (g_phaseSamples > 10) {
    double mean = g_phaseQSum / g_phaseSamples;
    double rmse = std::sqrt (std::max (0.0,
        g_phaseQSumSq/g_phaseSamples - 2.0*g_phaseQRef*mean + g_phaseQRef*g_phaseQRef));
    double sig  = std::sqrt (std::max (0.0, g_phaseQSumSq/g_phaseSamples - mean*mean));
    NS_LOG_UNCOND ("MPC Phase " << g_phaseNum << " [" << reason << "]"
        << "  qRef=" << (int)g_phaseQRef
        << "  mean=" << std::fixed << std::setprecision(1) << mean
        << "  RMSE=" << rmse << "  sigma=" << sig);
    g_phaseLog << g_phaseNum << "," << g_phaseStart << ","
               << g_phaseQRef << "," << mean << "," << rmse << "," << sig << "\n";
    g_phaseLog.flush ();
  }
  g_phaseQSum = 0; g_phaseQSumSq = 0; g_phaseSamples = 0;
  g_phaseStart = Simulator::Now ().GetSeconds ();
  g_phaseNum++;
  if (g_qdisc) g_phaseQRef = g_qdisc->GetQueueRef ();
}

static void ChangeQRef (double newRef, const char* reason)
{
  FlushPhase (reason);
  if (!g_qdisc) return;
  double old = g_qdisc->GetQueueRef ();
  g_qdisc->SetQueueRef (newRef);
  g_phaseQRef = newRef;
  NS_LOG_UNCOND ("MPC t=" << Simulator::Now ().GetSeconds ()
      << "s  qRef: " << (int)old << " -> " << (int)newRef
      << "  [bumpless: integral+deriv reset]");
  g_eventLog << Simulator::Now ().GetSeconds ()
             << ",qRef_change," << (int)old << "," << (int)newRef << "\n";
  g_eventLog.flush ();
}

static void FlowEvent (const char* msg, int delta, int total)
{
  FlushPhase (msg);
  NS_LOG_UNCOND ("MPC t=" << Simulator::Now ().GetSeconds ()
      << "s  " << msg << "  total=" << total);
  g_eventLog << Simulator::Now ().GetSeconds ()
             << ",flow_change," << delta << "," << total << "\n";
  g_eventLog.flush ();
}

int main (int argc, char *argv[])
{
  uint32_t    seed    = 1;
  double      simTime = 120.0;
  std::string outDir  = "";
  bool        pcap    = false;

  CommandLine cmd;
  cmd.AddValue ("seed",       "RNG seed",             seed);
  cmd.AddValue ("simTime",    "Simulation time (s)",  simTime);
  cmd.AddValue ("outDir",     "Output directory",     outDir);
  cmd.AddValue ("enablePcap", "Enable bottleneck pcap", pcap);
  cmd.Parse (argc, argv);

  RngSeedManager::SetSeed (seed * 100 + 7);
  RngSeedManager::SetRun  (seed);

  if (!outDir.empty ()) mkdir (outDir.c_str (), 0755);

  // ── Flow counts ──────────────────────────────────────────────────────────
  //  Base  : 120 BulkTCP + 60 OnOffTCP + 20 UDP = 200
  //  Extra : 30 BulkTCP  + 15 OnOffTCP + 5  UDP = 50 per pool
  const uint32_t BASE_BULK = 120, BASE_WEB = 60, BASE_UDP = 20;
  const uint32_t EX_BULK   =  30, EX_WEB  = 15, EX_UDP  =  5;
  const uint32_t nBase  = BASE_BULK + BASE_WEB + BASE_UDP;   // 200
  const uint32_t nPool  = EX_BULK   + EX_WEB   + EX_UDP;    // 50
  const uint32_t nTotal = nBase + nPool * 3;                 // 350

  NS_LOG_UNCOND ("═══════════════════════════════════════════════════════");
  NS_LOG_UNCOND (" Hybrid MPC+PID+RLS Dynamic  seed=" << seed);
  NS_LOG_UNCOND (" Kp=0.25  Ki=0.35  Kd=0.010  Horizon=25  MpcGain=0.005  PidWeight=0.7");
  NS_LOG_UNCOND (" Base : " << BASE_BULK << " BulkTCP + " << BASE_WEB
      << " OnOffTCP + " << BASE_UDP << " UDP = " << nBase);
  NS_LOG_UNCOND (" Extra: " << EX_BULK << " BulkTCP + " << EX_WEB
      << " OnOffTCP + " << EX_UDP << " UDP = " << nPool << " per pool x3");
  NS_LOG_UNCOND (" Total nodes: " << nTotal);
  NS_LOG_UNCOND ("═══════════════════════════════════════════════════════");

  NodeContainer senders, receivers, routers;
  senders.Create (nTotal); receivers.Create (nTotal); routers.Create (2);
  InternetStackHelper stack; stack.InstallAll ();

  // ── Bottleneck (ITU-T G.114: 45 Mbps, 50ms) ─────────────────────────────
  PointToPointHelper bn;
  bn.SetDeviceAttribute  ("DataRate", StringValue ("45Mbps"));
  bn.SetChannelAttribute ("Delay",    StringValue ("50ms"));
  NetDeviceContainer bnd = bn.Install (routers.Get(0), routers.Get(1));

  // ── AQM: Hybrid MPC+PID+RLS (optimised gains) ───────────────────────────
  TrafficControlHelper tch;
  tch.SetRootQueueDisc ("ns3::MPCQueueDisc",
      "QueueRef",         DoubleValue   (500.0),
      "Kp",               DoubleValue   (0.25),
      "Ki",               DoubleValue   (0.35),
      "Kd",               DoubleValue   (0.010),
      "Horizon",          UintegerValue (25),
      "ControlWeight",    DoubleValue   (0.5),
      "MpcGain",          DoubleValue   (0.005),
      "PidWeight",        DoubleValue   (0.7),
      "ForgettingFactor", DoubleValue   (0.98));
  QueueDiscContainer qdiscs = tch.Install (bnd.Get(0));
  g_qdisc = DynamicCast<MPCQueueDisc> (qdiscs.Get(0));
  NS_ASSERT_MSG (g_qdisc, "DynamicCast to MPCQueueDisc failed");
  if (pcap) bn.EnablePcap ("mpc-bottleneck", bnd.Get(0), true);

  // ── Output files ─────────────────────────────────────────────────────────
  auto H   = std::getenv ("HOME");
  auto mkp = [&](const char* fn) {
    return outDir.empty () ? (H ? std::string(H)+"/"+fn : std::string(fn))
                           : outDir + "/" + fn;
  };
  g_qLog.open     (mkp ("mpc-queue-length.csv")); g_qLog     << "time,queue\n";
  g_eventLog.open (mkp ("mpc-events.csv"));        g_eventLog << "time,event,param1,param2\n";
  g_phaseLog.open (mkp ("mpc-phase-stats.csv"));
  g_phaseLog << "phase,t_start,qRef,mean_q,rmse,sigma\n";
  qdiscs.Get(0)->TraceConnectWithoutContext ("PacketsInQueue", MakeCallback (&QueueTrace));

  // ── Addressing ───────────────────────────────────────────────────────────
  Ipv4AddressHelper addr;
  addr.SetBase ("10.1.0.0", "255.255.255.0");
  addr.Assign (bnd); addr.NewNetwork ();

  Ptr<UniformRandomVariable> startRv = CreateObject<UniformRandomVariable> ();
  startRv->SetAttribute ("Min", DoubleValue (0.0));
  startRv->SetAttribute ("Max", DoubleValue (20.0));

  Ptr<ParetoRandomVariable> paretoRv = CreateObject<ParetoRandomVariable> ();
  paretoRv->SetAttribute ("Shape", DoubleValue (1.2));
  paretoRv->SetAttribute ("Scale", DoubleValue (50000.0));

  PointToPointHelper ac;
  ac.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  std::vector<Ipv4Address> ra (nTotal);

  for (uint32_t i = 0; i < nTotal; i++) {
    if      (i % 3 == 0) ac.SetChannelAttribute ("Delay", StringValue ("5ms"));
    else if (i % 3 == 1) ac.SetChannelAttribute ("Delay", StringValue ("20ms"));
    else                  ac.SetChannelAttribute ("Delay", StringValue ("50ms"));
    NetDeviceContainer ld = ac.Install (senders.Get(i),   routers.Get(0));
    addr.Assign (ld); addr.NewNetwork ();
    NetDeviceContainer rd = ac.Install (routers.Get(1), receivers.Get(i));
    Ipv4InterfaceContainer ri = addr.Assign (rd); addr.NewNetwork ();
    ra[i] = ri.GetAddress (1);
  }
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // ── Application installers ───────────────────────────────────────────────
  auto instBulk = [&](uint32_t i, double t0, double tS) {
    uint16_t p = 5000 + (uint16_t)i;
    PacketSinkHelper s ("ns3::TcpSocketFactory",
        InetSocketAddress (Ipv4Address::GetAny (), p));
    auto sa = s.Install (receivers.Get(i));
    sa.Start (Seconds(0)); sa.Stop (Seconds(tS + 5));
    BulkSendHelper b ("ns3::TcpSocketFactory",
        InetSocketAddress (ra[i], p));
    uint32_t fsz = (uint32_t)std::min (1e8, std::max (65536.0, paretoRv->GetValue ()));
    b.SetAttribute ("MaxBytes", UintegerValue (fsz));
    auto ba = b.Install (senders.Get(i));
    ba.Start (Seconds(t0)); ba.Stop (Seconds(tS));
  };

  auto instWeb = [&](uint32_t i, double t0, double tS) {
    uint16_t p = 5000 + (uint16_t)i;
    PacketSinkHelper s ("ns3::TcpSocketFactory",
        InetSocketAddress (Ipv4Address::GetAny (), p));
    auto sa = s.Install (receivers.Get(i));
    sa.Start (Seconds(0)); sa.Stop (Seconds(tS + 5));
    OnOffHelper w ("ns3::TcpSocketFactory", InetSocketAddress (ra[i], p));
    w.SetAttribute ("DataRate",   StringValue ("5Mbps"));
    w.SetAttribute ("PacketSize", UintegerValue (1000));
    w.SetAttribute ("OnTime",  StringValue ("ns3::ExponentialRandomVariable[Mean=1.0]"));
    w.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable[Mean=2.0]"));
    auto wa = w.Install (senders.Get(i));
    wa.Start (Seconds(t0)); wa.Stop (Seconds(tS));
  };

  auto instUdp = [&](uint32_t i, double t0, double tS) {
    uint16_t p = 5000 + (uint16_t)i;
    PacketSinkHelper s ("ns3::UdpSocketFactory",
        InetSocketAddress (Ipv4Address::GetAny (), p));
    auto sa = s.Install (receivers.Get(i));
    sa.Start (Seconds(0)); sa.Stop (Seconds(tS + 5));
    OnOffHelper v ("ns3::UdpSocketFactory", InetSocketAddress (ra[i], p));
    v.SetAttribute ("DataRate",   StringValue ("2Mbps"));
    v.SetAttribute ("PacketSize", UintegerValue (1200));
    v.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1e9]"));
    v.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
    auto va = v.Install (senders.Get(i));
    va.Start (Seconds(t0)); va.Stop (Seconds(tS));
  };

  // ── BASE flows (0..199): 120 BulkTCP + 60 OnOffTCP + 20 UDP ────────────
  // First 60 BulkTCP (i=0..59) stop at t=60s (-60 flow event)
  for (uint32_t i = 0; i < nBase; i++) {
    double t0 = startRv->GetValue ();
    if (i < BASE_BULK) {
      double tStop = (i < 60) ? 60.0 : simTime;
      instBulk (i, t0, tStop);
    } else if (i < BASE_BULK + BASE_WEB) {
      instWeb (i, t0, simTime);
    } else {
      instUdp (i, t0, simTime);
    }
  }

  // ── EXTRA-A (200..249): 30 BulkTCP + 15 OnOffTCP + 5 UDP  [t=25s] ──────
  for (uint32_t i = nBase; i < nBase + nPool; i++) {
    uint32_t li = i - nBase;
    if      (li < EX_BULK)           instBulk (i, 25.0, simTime);
    else if (li < EX_BULK + EX_WEB)  instWeb  (i, 25.0, simTime);
    else                              instUdp  (i, 25.0, simTime);
  }

  // ── EXTRA-B (250..299): 30 BulkTCP + 15 OnOffTCP + 5 UDP  [t=50s..85s] ─
  for (uint32_t i = nBase + nPool; i < nBase + 2*nPool; i++) {
    uint32_t li = i - nBase - nPool;
    if      (li < EX_BULK)           instBulk (i, 50.0, 85.0);
    else if (li < EX_BULK + EX_WEB)  instWeb  (i, 50.0, 85.0);
    else                              instUdp  (i, 50.0, 85.0);
  }

  // ── EXTRA-C (300..349): 30 BulkTCP + 15 OnOffTCP + 5 UDP  [t=110s] ─────
  for (uint32_t i = nBase + 2*nPool; i < nTotal; i++) {
    uint32_t li = i - nBase - 2*nPool;
    if      (li < EX_BULK)           instBulk (i, 110.0, simTime);
    else if (li < EX_BULK + EX_WEB)  instWeb  (i, 110.0, simTime);
    else                              instUdp  (i, 110.0, simTime);
  }

  // ── Dynamic event schedule ───────────────────────────────────────────────
  Simulator::Schedule (Seconds (25.0),  &FlowEvent, "+50 (30B+15W+5U) Extra-A start", +50, 250);
  Simulator::Schedule (Seconds (40.0),  &ChangeQRef, 300.0, "qRef->300");
  Simulator::Schedule (Seconds (50.0),  &FlowEvent, "+50 (30B+15W+5U) Extra-B start", +50, 300);
  Simulator::Schedule (Seconds (60.0),  &FlowEvent, "-60 BulkTCP base stopped",       -60, 240);
  Simulator::Schedule (Seconds (75.0),  &ChangeQRef, 100.0, "qRef->100 stress-test");
  Simulator::Schedule (Seconds (85.0),  &FlowEvent, "-50 Extra-B stopped",            -50, 190);
  Simulator::Schedule (Seconds (95.0),  &ChangeQRef, 700.0, "qRef->700");
  Simulator::Schedule (Seconds (110.0), &FlowEvent, "+50 (30B+15W+5U) Extra-C start", +50, 240);
  Simulator::Schedule (Seconds (simTime - 0.01), &FlushPhase, "end");

  FlowMonitorHelper fmh; Ptr<FlowMonitor> fm = fmh.InstallAll ();

  NS_LOG_UNCOND ("Running " << simTime << "s ...");
  Simulator::Stop (Seconds (simTime));
  Simulator::Run  ();

  fm->SerializeToXmlFile (
      outDir.empty () ? "mpc-flow-dyn.xml" : outDir + "/flow-results.xml",
      true, true);
  Simulator::Destroy ();
  g_qLog.close (); g_eventLog.close (); g_phaseLog.close ();
  NS_LOG_UNCOND ("MPC done.");
  return 0;
}
