/**
 * main.cc
 * =======================================================
 * Heterogeneous dumbbell topology for evaluating the
 * Hybrid RLS-MPC + PID Active Queue Management controller.
 *
 * Topology:
 *   nFlows senders --+                   +-- nFlows receivers
 *                    +-- R0 --[45Mbps]-- R1
 *   (100Mbps access, |   ^               |
 *   RTT diverse)     |   AQM here        |
 *
 * Traffic mix:
 *   60% BulkSend TCP  -- elephant flows (Pareto file sizes)
 *   30% OnOff TCP     -- web-like mice flows (Exp ON/OFF)
 *   10% OnOff UDP     -- unresponsive video streams
 *
 * RTT diversity:
 *   Flow i%3==0 : access delay = 10ms
 *   Flow i%3==1 : access delay = 50ms
 *   Flow i%3==2 : access delay = 150ms
 *
 * Outputs:
 *   queue-length.csv        -- queue length over time
 *   mpc-hybrid-results.csv  -- controller internals every Ts
 *   flow-results.xml        -- per-flow throughput/delay/loss
 * =======================================================
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("HybridMPCDumbbell");

/* ── Global queue-length log ─────────────────────────────── */
static std::ofstream g_queueLog;

static void
QueueLengthTrace (uint32_t /* oldVal */, uint32_t newVal)
{
  g_queueLog << Simulator::Now ().GetSeconds ()
             << "," << newVal << "\n";
}

/* ── main ────────────────────────────────────────────────── */
int
main (int argc, char *argv[])
{
  /* ── Simulation parameters ─────────────────────────────── */
  uint32_t nFlows       = 200;
  double   simTime      = 120.0;
  double   bottleRate   = 45.0;
  double   bottleDelay  = 100.0;
  double   accessRate   = 100.0;
  uint32_t queueRef     = 500;

  CommandLine cmd;
  cmd.AddValue ("nFlows",  "Number of flows",        nFlows);
  cmd.AddValue ("simTime", "Simulation time (s)",    simTime);
  cmd.AddValue ("qRef",    "Target queue (packets)", queueRef);
  cmd.Parse (argc, argv);

  /* ── Traffic split ──────────────────────────────────────── */
  uint32_t nBulk = static_cast<uint32_t> (nFlows * 0.60);
  uint32_t nWeb  = static_cast<uint32_t> (nFlows * 0.30);
  uint32_t nUdp  = nFlows - nBulk - nWeb;

  NS_LOG_UNCOND ("Flows: Bulk=" << nBulk
                 << " Web=" << nWeb
                 << " UDP=" << nUdp);

  /* ── Nodes ──────────────────────────────────────────────── */
  NodeContainer senders, receivers;
  senders.Create   (nFlows);
  receivers.Create (nFlows);

  NodeContainer routers;
  routers.Create (2);

  InternetStackHelper stack;
  stack.InstallAll ();

  /* ── Link helpers ───────────────────────────────────────── */
  PointToPointHelper accessLink;
  accessLink.SetDeviceAttribute ("DataRate",
    StringValue (std::to_string ((int)accessRate) + "Mbps"));

  PointToPointHelper bottleneckLink;
  bottleneckLink.SetDeviceAttribute ("DataRate",
    StringValue (std::to_string ((int)bottleRate) + "Mbps"));
  bottleneckLink.SetChannelAttribute ("Delay",
    StringValue (std::to_string ((int)bottleDelay) + "ms"));

  /* ── Bottleneck device with MPC queue disc ──────────────── */
  NetDeviceContainer bottleneckDev =
      bottleneckLink.Install (routers.Get (0), routers.Get (1));

  TrafficControlHelper tch;
  tch.SetRootQueueDisc (
    "ns3::MPCQueueDisc",
    "QueueRef",        DoubleValue  (static_cast<double>(queueRef)),
    "SamplingTime",    DoubleValue  (0.01),
    "Horizon",         UintegerValue(20),
    "ControlWeight",   DoubleValue  (0.1),
    "MpcGain",         DoubleValue  (0.4),
    "PidWeight",       DoubleValue  (0.3),
    "Kp",              DoubleValue  (0.04),
    "Ki",              DoubleValue  (0.004),
    "Kd",              DoubleValue  (0.006),
    "ForgettingFactor",DoubleValue  (0.99)
  );

  QueueDiscContainer qdiscs =
      tch.Install (bottleneckDev.Get (0));

  /* ── Queue length trace ─────────────────────────────────── */
  g_queueLog.open ("queue-length.csv");
  g_queueLog << "time_s,queue_pkts\n";

  Ptr<QueueDisc> qd = qdiscs.Get (0);
  qd->TraceConnectWithoutContext (
      "PacketsInQueue",
      MakeCallback (&QueueLengthTrace));

  /* ── IP addressing ──────────────────────────────────────── */
  Ipv4AddressHelper ipHelper;
  ipHelper.SetBase ("10.0.0.0", "255.255.255.0");
  ipHelper.Assign  (bottleneckDev);
  ipHelper.NewNetwork ();

  /* ── Random variable helpers ────────────────────────────── */
  Ptr<UniformRandomVariable> startRv =
      CreateObject<UniformRandomVariable> ();
  startRv->SetAttribute ("Min", DoubleValue (0.0));
  startRv->SetAttribute ("Max", DoubleValue (30.0));

  Ptr<ParetoRandomVariable> paretoRv =
      CreateObject<ParetoRandomVariable> ();
  paretoRv->SetAttribute ("Shape", DoubleValue (1.5));
  paretoRv->SetAttribute ("Scale", DoubleValue (100000));

  uint16_t basePort = 5000;

  /* ── Per-flow setup ─────────────────────────────────────── */
  for (uint32_t i = 0; i < nFlows; i++)
  {
    /* RTT diversity */
    std::string accDelay;
    if      (i % 3 == 0) accDelay = "10ms";
    else if (i % 3 == 1) accDelay = "50ms";
    else                  accDelay = "150ms";
    accessLink.SetChannelAttribute ("Delay", StringValue (accDelay));

    /* sender(i) <-> router0 */
    NetDeviceContainer devSR =
        accessLink.Install (senders.Get (i), routers.Get (0));
    ipHelper.Assign   (devSR);
    ipHelper.NewNetwork ();

    /* router1 <-> receiver(i) */
    NetDeviceContainer devRR =
        accessLink.Install (routers.Get (1), receivers.Get (i));
    ipHelper.Assign   (devRR);
    ipHelper.NewNetwork ();

    uint16_t port   = basePort + i;
    double   tStart = startRv->GetValue ();

    Ipv4Address recvAddr =
        receivers.Get (i)->GetObject<Ipv4> ()
                         ->GetAddress (1, 0).GetLocal ();

    /* ── BULK TCP ───────────────────────────────────────── */
    if (i < nBulk)
    {
      PacketSinkHelper tcpSink (
          "ns3::TcpSocketFactory",
          InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp =
          tcpSink.Install (receivers.Get (i));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop  (Seconds (simTime));

      uint32_t fileSize =
          static_cast<uint32_t> (paretoRv->GetValue ());
      fileSize = std::max (fileSize, 65536u);
      fileSize = std::min (fileSize, 100000000u);

      BulkSendHelper tcpSrc (
          "ns3::TcpSocketFactory",
          InetSocketAddress (recvAddr, port));
      tcpSrc.SetAttribute ("MaxBytes", UintegerValue (fileSize));
      ApplicationContainer srcApp =
          tcpSrc.Install (senders.Get (i));
      srcApp.Start (Seconds (tStart));
      srcApp.Stop  (Seconds (simTime));
    }

    /* ── WEB TCP ────────────────────────────────────────── */
    else if (i < nBulk + nWeb)
    {
      PacketSinkHelper tcpSink (
          "ns3::TcpSocketFactory",
          InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp =
          tcpSink.Install (receivers.Get (i));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop  (Seconds (simTime));

      OnOffHelper webSrc (
          "ns3::TcpSocketFactory",
          InetSocketAddress (recvAddr, port));
      webSrc.SetAttribute ("DataRate",
          DataRateValue (DataRate ("5Mbps")));
      webSrc.SetAttribute ("PacketSize",
          UintegerValue (1000));
      webSrc.SetAttribute ("OnTime",
          StringValue ("ns3::ExponentialRandomVariable[Mean=1.0]"));
      webSrc.SetAttribute ("OffTime",
          StringValue ("ns3::ExponentialRandomVariable[Mean=2.0]"));
      ApplicationContainer srcApp =
          webSrc.Install (senders.Get (i));
      srcApp.Start (Seconds (tStart));
      srcApp.Stop  (Seconds (simTime));
    }

    /* ── UDP VIDEO ──────────────────────────────────────── */
    else
    {
      /* UDP sink — MUST match UDP source */
      PacketSinkHelper udpSink (
          "ns3::UdpSocketFactory",
          InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp =
          udpSink.Install (receivers.Get (i));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop  (Seconds (simTime));

      OnOffHelper videoSrc (
          "ns3::UdpSocketFactory",
          InetSocketAddress (recvAddr, port));
      videoSrc.SetAttribute ("DataRate",
          DataRateValue (DataRate ("2Mbps")));
      videoSrc.SetAttribute ("PacketSize",
          UintegerValue (1200));
      videoSrc.SetAttribute ("OnTime",
          StringValue ("ns3::ExponentialRandomVariable[Mean=2.0]"));
      videoSrc.SetAttribute ("OffTime",
          StringValue ("ns3::ExponentialRandomVariable[Mean=1.0]"));
      ApplicationContainer srcApp =
          videoSrc.Install (senders.Get (i));
      srcApp.Start (Seconds (tStart));
      srcApp.Stop  (Seconds (simTime));
    }
  }

  /* ── Routing ────────────────────────────────────────────── */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  /* ── FlowMonitor ────────────────────────────────────────── */
  FlowMonitorHelper flowMonHelper;
  Ptr<FlowMonitor> flowMon = flowMonHelper.InstallAll ();

  /* ── Run ────────────────────────────────────────────────── */
  NS_LOG_UNCOND ("Starting simulation for " << simTime << " s ...");
  Simulator::Stop (Seconds (simTime));
  Simulator::Run  ();

  /* ── Output ─────────────────────────────────────────────── */
  flowMon->SerializeToXmlFile ("flow-results.xml", true, true);
  Simulator::Destroy ();
  g_queueLog.close ();

  NS_LOG_UNCOND ("Done.");
  NS_LOG_UNCOND ("  queue-length.csv      -- queue over time");
  NS_LOG_UNCOND ("  mpc-hybrid-results.csv -- controller internals");
  NS_LOG_UNCOND ("  flow-results.xml      -- per-flow statistics");

  return 0;
}
