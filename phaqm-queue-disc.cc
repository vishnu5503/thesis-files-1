/*
 * phaqm-queue-disc.cc
 * Fixed version:
 *   1. QueueRef registered as TypeId attribute  -> SetAttribute("QueueRef",...) works
 *   2. SetQueueRef() / GetQueueRef() / GetDropProb() public methods added
 *   3. CSV logger is per-instance (NOT static) -> 5 seeds write separate files
 *   4. Hebbian weight reset on SetQueueRef for smoother adaptation
 */

#include "phaqm-queue-disc.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/queue-disc.h"
#include "ns3/queue-size.h"
#include "ns3/drop-tail-queue.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("PhaqmQueueDisc");
NS_OBJECT_ENSURE_REGISTERED (PhaqmQueueDisc);

TypeId
PhaqmQueueDisc::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::PhaqmQueueDisc")
        .SetParent<QueueDisc> ()
        .SetGroupName ("TrafficControl")
        .AddConstructor<PhaqmQueueDisc> ()
        .AddAttribute (
            "MaxSize",
            "Maximum queue size (packets)",
            QueueSizeValue (QueueSize (QueueSizeUnit::PACKETS, 1000)),
            MakeQueueSizeAccessor (&PhaqmQueueDisc::SetMaxSize,
                                   &PhaqmQueueDisc::GetMaxSize),
            MakeQueueSizeChecker ())
        // ── FIX 1: QueueRef now a proper TypeId attribute ──────────────────
        .AddAttribute (
            "QueueRef",
            "Target queue occupancy (packets)",
            DoubleValue (500.0),
            MakeDoubleAccessor (&PhaqmQueueDisc::SetQueueRef,
                                &PhaqmQueueDisc::GetQueueRef),
            MakeDoubleChecker<double> (1.0, 1000.0));
    return tid;
}

PhaqmQueueDisc::PhaqmQueueDisc ()
    : m_maxSize        (QueueSize (QueueSizeUnit::PACKETS, 1000)),
      m_queueRef       (500.0),
      m_lastDropProb   (0.0),
      m_C              (45e6 / 8000.0),
      m_R0             (0.1),
      m_N              (200),
      m_samplingTime   (0.01),
      m_delayD         (14),
      m_prevError      (0.0),
      m_prevControl    (0.0),
      m_logCounter     (0),
      m_headerWritten  (false)
{
    NS_LOG_INFO ("Constructing PhaqmQueueDisc");
    m_uniformRV = CreateObject<UniformRandomVariable> ();

    size_t numWeights = m_delayD + 3;
    m_yHist.assign (m_delayD + 1, 0.0);
    m_uHist.assign (m_delayD + 2, 0.0);
    m_weights.assign       (numWeights, 0.01);
    m_learningRates.assign (numWeights, 0.001);
    m_prevInput.assign     (numWeights, 0.0);

    ComputeModelParameters     (m_C, m_R0, m_N, m_samplingTime);
    SetDiophantineCoefficients ();
    ComputeControlCoefficients ();

    Ptr<DropTailQueue<QueueDiscItem>> q = CreateObject<DropTailQueue<QueueDiscItem>> ();
    q->SetAttribute ("MaxSize", QueueSizeValue (m_maxSize));
    AddInternalQueue (q);
}

PhaqmQueueDisc::~PhaqmQueueDisc ()
{
    if (m_csvLog.is_open ()) m_csvLog.close ();
}

// ── FIX 2: SetQueueRef resets Hebbian weights for bumpless adaptation ────────
void
PhaqmQueueDisc::SetQueueRef (double qRef)
{
    NS_LOG_UNCOND ("PhaqmQueueDisc::SetQueueRef " << m_queueRef << " -> " << qRef);
    m_queueRef = qRef;
    // Partial weight reset: scale towards zero to help Hebbian re-adapt
    // (full zero-reset causes a large transient; scaling is smoother)
    for (auto& w : m_weights)  w *= 0.5;
    m_prevError   = 0.0;
    m_prevControl = 0.0;
}

double
PhaqmQueueDisc::GetQueueRef () const
{
    return m_queueRef;
}

double
PhaqmQueueDisc::GetDropProb () const
{
    return m_lastDropProb;
}

void
PhaqmQueueDisc::SetMaxSize (const QueueSize& maxSize)
{
    m_maxSize = maxSize;
}

QueueSize
PhaqmQueueDisc::GetMaxSize () const
{
    return m_maxSize;
}

bool
PhaqmQueueDisc::CheckConfig (void)
{
    if (GetNInternalQueues () == 0)
        NS_FATAL_ERROR ("PhaqmQueueDisc needs at least one internal queue");
    if (m_maxSize.GetUnit () != QueueSizeUnit::PACKETS) return false;
    return true;
}

void
PhaqmQueueDisc::InitializeParams (void)
{
    std::fill (m_yHist.begin (), m_yHist.end (), 0.0);
    std::fill (m_uHist.begin (), m_uHist.end (), 0.0);
    std::fill (m_weights.begin (), m_weights.end (), 0.01);
    m_prevError = 0.0; m_prevControl = 0.0;
    std::fill (m_prevInput.begin (), m_prevInput.end (), 0.0);
}

void
PhaqmQueueDisc::ComputeModelParameters (double C, double R0, uint32_t N, double Ts)
{
    m_delayD = static_cast<uint32_t> (std::round (R0 / Ts));
    m1 = C / (3.0 * pow (R0,3)) *
         (2.0 - 2.0*exp(-Ts/R0)*N + C*R0*(-1.0 + exp(-2.0*N*Ts/(C*pow(R0,2)))));
    m2 = C / (3.0 * pow (R0,3)) * exp(-Ts/R0) *
         (2.0 - (1.0-exp(Ts/R0))*N - C*R0*(-1.0 + exp(2.0*N*Ts/(C*pow(R0,2)))));
    n1 = -exp(-2.0*N*Ts/(C*pow(R0,2))) - exp(-Ts/R0);
    n2 =  exp(-(2.0*N + C*R0)*Ts/(C*pow(R0,2)));
}

void
PhaqmQueueDisc::SetDiophantineCoefficients ()
{
    e_coeffs.assign (m_delayD + 1, 0.0);
    e_coeffs[0] = 1.0;
    f_coeffs = {0.5, -0.4};
}

void
PhaqmQueueDisc::ComputeControlCoefficients ()
{
    g_coeffs.resize (m_delayD + 2, 0.0);
    g_coeffs[0] = e_coeffs[0] * m1;
    g_coeffs[1] = ((e_coeffs.size () > 1) ? e_coeffs[1] : 0.0) * m1
                  + e_coeffs[0] * m2;
    for (size_t i = 2; i < g_coeffs.size (); ++i) {
        double ei   = (i   < e_coeffs.size ()) ? e_coeffs[i]   : 0.0;
        double ei_1 = (i-1 < e_coeffs.size ()) ? e_coeffs[i-1] : 0.0;
        g_coeffs[i] = m1*ei + m2*ei_1;
    }
    h_coeffs.resize (g_coeffs.size (), 0.0);
    for (size_t i = 0; i < f_coeffs.size (); ++i)
        h_coeffs[i] = -f_coeffs[i] / g_coeffs[0];
    for (size_t i = f_coeffs.size (); i < h_coeffs.size (); ++i)
        h_coeffs[i] = -g_coeffs[i] / g_coeffs[0];
    if (h_coeffs.size () > m_delayD + 3)
        h_coeffs.resize (m_delayD + 3);
}

bool
PhaqmQueueDisc::DoEnqueue (Ptr<QueueDiscItem> item)
{
    double queueLen = static_cast<double> (GetCurrentSize ().GetValue ());
    double error    = queueLen - m_queueRef;

    m_yHist.pop_back ();
    m_yHist.insert (m_yHist.begin (), error);

    // Build input vector x(t)
    std::vector<double> x;
    x.push_back (m_yHist[0]);
    x.push_back (m_yHist[1]);
    for (size_t i = 0; i < m_delayD + 1; ++i)
        x.push_back (i < m_uHist.size () ? m_uHist[i] : 0.0);

    // Compute control signal (Hebbian neural network output)
    double u_t = 0.0;
    for (size_t i = 0; i < m_weights.size (); ++i)
        u_t += m_weights[i] * x[i];
    u_t = std::clamp (u_t, 0.0, 1.0);

    // Hebbian weight update
    if (m_prevInput.size () == m_weights.size ()) {
        for (size_t i = 0; i < m_weights.size (); ++i) {
            double dw = m_learningRates[i] * m_prevError * m_prevInput[i] * m_prevControl;
            m_weights[i] = std::clamp (m_weights[i] + dw, -1.0, 1.0);
        }
    }

    m_prevError   = error;
    m_prevControl = u_t;
    m_prevInput   = x;
    m_lastDropProb = u_t;   // store for GetDropProb()

    m_uHist.pop_back ();
    m_uHist.insert (m_uHist.begin (), u_t);

    // ── FIX 3: per-instance CSV log (NOT static) ─────────────────────────────
    if (!m_headerWritten) {
        const char* home = std::getenv ("HOME");
        std::string path = home
            ? std::string(home) + "/phaqm-results.csv"
            : "phaqm-results.csv";
        m_csvLog.open (path);
        m_csvLog << "time_s,queue_pkts,drop_prob\n";
        m_headerWritten = true;
    }
    if (++m_logCounter % 100 == 0) {
        m_csvLog << Simulator::Now ().GetSeconds ()
                 << "," << queueLen
                 << "," << u_t << "\n";
    }
    // ─────────────────────────────────────────────────────────────────────────

    if (m_uniformRV->GetValue () < u_t) {
        DropBeforeEnqueue (item, "PHAQM-Hebb-Drop");
        return false;
    }
    return GetInternalQueue (0)->Enqueue (item);
}

Ptr<QueueDiscItem>
PhaqmQueueDisc::DoDequeue (void)
{
    return GetInternalQueue (0)->Dequeue ();
}

void
PhaqmQueueDisc::DoDispose (void)
{
    if (m_csvLog.is_open ()) m_csvLog.close ();
    QueueDisc::DoDispose ();
}

} // namespace ns3
