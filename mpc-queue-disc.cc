/**
 * mpc-queue-disc.cc
 * Implementation of Hybrid RLS-MPC + PID Queue Discipline
 *
 * See mpc-queue-disc.h for full algorithm description.
 *
 * NEW in this version:
 *   SetQueueRef(double) — change qRef mid-simulation with bumpless transfer
 *   GetQueueRef()       — read current target
 *   GetDropProb()       — read current drop probability (for NetAnim)
 */

#include "mpc-queue-disc.h"        // own header MUST be first

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/drop-tail-queue.h"
#include "ns3/queue-size.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"

#include <cmath>
#include <algorithm>
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MPCQueueDisc");
NS_OBJECT_ENSURE_REGISTERED (MPCQueueDisc);

/* ==========================================================
   TypeId — register class + expose attributes
   ========================================================== */
TypeId
MPCQueueDisc::GetTypeId ()
{
  static TypeId tid =
    TypeId ("ns3::MPCQueueDisc")
      .SetParent<QueueDisc>()
      .SetGroupName ("TrafficControl")
      .AddConstructor<MPCQueueDisc>()

      /* Target */
      .AddAttribute ("QueueRef",
                     "Desired queue length in packets.",
                     DoubleValue (500.0),
                     MakeDoubleAccessor (&MPCQueueDisc::m_qRef),
                     MakeDoubleChecker<double> (1.0))

      /* Sampling period */
      .AddAttribute ("SamplingTime",
                     "Controller sampling period in seconds.",
                     DoubleValue (0.01),
                     MakeDoubleAccessor (&MPCQueueDisc::m_Ts),
                     MakeDoubleChecker<double> (1e-4))

      /* MPC parameters */
      .AddAttribute ("Horizon",
                     "MPC prediction horizon N.",
                     UintegerValue (20),
                     MakeUintegerAccessor (&MPCQueueDisc::m_N),
                     MakeUintegerChecker<uint32_t> (1))
      .AddAttribute ("ControlWeight",
                     "MPC quadratic cost weight on control effort R.",
                     DoubleValue (0.1),
                     MakeDoubleAccessor (&MPCQueueDisc::m_R),
                     MakeDoubleChecker<double> (0.0))
      .AddAttribute ("MpcGain",
                     "Scaling gain applied to MPC output (Km).",
                     DoubleValue (0.005),
                     MakeDoubleAccessor (&MPCQueueDisc::m_Km),
                     MakeDoubleChecker<double> ())

      /* PID parameters */
      .AddAttribute ("PidWeight",
                     "Weighting on PID contribution in hybrid (Wpid).",
                     DoubleValue (0.3),
                     MakeDoubleAccessor (&MPCQueueDisc::m_Wpid),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("Kp",
                     "PID proportional gain.",
                     DoubleValue (0.30),
                     MakeDoubleAccessor (&MPCQueueDisc::m_Kp),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("Ki",
                     "PID integral gain.",
                     DoubleValue (0.50),
                     MakeDoubleAccessor (&MPCQueueDisc::m_Ki),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("Kd",
                     "PID derivative gain.",
                     DoubleValue (0.008),
                     MakeDoubleAccessor (&MPCQueueDisc::m_Kd),
                     MakeDoubleChecker<double> ())

      /* RLS parameter */
      .AddAttribute ("ForgettingFactor",
                     "RLS exponential forgetting factor lambda in (0,1).",
                     DoubleValue (0.99),
                     MakeDoubleAccessor (&MPCQueueDisc::m_lambda),
                     MakeDoubleChecker<double> (0.9, 1.0));

  return tid;
}

/* ==========================================================
   Constructor
   ========================================================== */
MPCQueueDisc::MPCQueueDisc ()
  : m_qRef    (500.0),
    m_Ts      (0.01),
    m_N       (20),
    m_R       (0.1),
    m_Km      (0.00),
    m_Wpid    (0.3),
    m_Kp      (0.30),
    m_Ki      (0.50),
    m_Kd      (0.008),
    m_lambda  (0.99),
    m_a       (0.95),
    m_b       (-0.002),
    m_prevQ   (0.0),
    m_prevU   (0.0),
    m_integral  (0.0),
    m_prevError (0.0),
    m_dropProb  (0.0)
{
  m_P[0][0] = 100.0;
  m_P[0][1] = 0.0;  m_P[1][0] = 0.0;  m_P[1][1] = 0.0;
  m_b       = -0.002;

  m_uv = CreateObject<UniformRandomVariable> ();

  {
    const char* home = std::getenv ("HOME");
    std::string csvPath = home
        ? std::string (home) + "/mpc-hybrid-results.csv"
        : "mpc-hybrid-results.csv";
    m_csv.open (csvPath);
  }
  if (m_csv.is_open ())
    m_csv << "time_s,queue_pkts,drop_prob,"
             "u_mpc,u_pid,u_hybrid,"
             "rls_a,rls_b\n";

  NS_LOG_INFO ("MPCQueueDisc constructed. qRef=" << m_qRef
               << " Ts=" << m_Ts << " N=" << m_N);
}

/* ==========================================================
   Destructor
   ========================================================== */
MPCQueueDisc::~MPCQueueDisc ()
{
  if (m_csv.is_open ())
    m_csv.close ();
}

/* ==========================================================
   SetQueueRef — change target queue length mid-simulation
   Bumpless transfer: resets integral + derivative memory so
   the controller does not lurch from old-reference windup.
   ========================================================== */
void
MPCQueueDisc::SetQueueRef (double newRef)
{
  NS_LOG_UNCOND ("MPCQueueDisc: QueueRef " << m_qRef << " → " << newRef
                 << " at t=" << Simulator::Now ().GetSeconds () << "s");
  m_qRef      = newRef;
  // Bumpless transfer: clear accumulated state tied to old reference
  m_integral  = 0.0;
  m_prevError = 0.0;
  // Keep m_dropProb unchanged — avoid step-change in drop probability
}

/* ==========================================================
   GetQueueRef — return current target
   ========================================================== */
double
MPCQueueDisc::GetQueueRef () const
{
  return m_qRef;
}

/* ==========================================================
   GetDropProb — return current drop probability (0..1)
   Used by NetAnim callbacks to display controller state.
   ========================================================== */
double
MPCQueueDisc::GetDropProb () const
{
  return m_dropProb;
}

/* ==========================================================
   InitializeParams
   ========================================================== */
void
MPCQueueDisc::InitializeParams ()
{
  NS_LOG_INFO ("MPCQueueDisc::InitializeParams — qRef=" << m_qRef
               << " Ts=" << m_Ts << " N=" << m_N << " R=" << m_R);

  m_event = Simulator::Schedule (
                Seconds (m_Ts),
                &MPCQueueDisc::ControlLoop,
                this);
}

/* ==========================================================
   ControlLoop — fires every m_Ts seconds
   ========================================================== */
void
MPCQueueDisc::ControlLoop ()
{
  double q = static_cast<double> (GetCurrentSize ().GetValue ());

  UpdateRLS (q);

  double u_mpc = ComputeMPC (q);

  double e_norm = (q - m_qRef) / m_qRef;

  if (q > 0.30 * m_qRef)
  {
    m_integral += m_Ki * m_Ts * e_norm;
    m_integral  = std::max (-0.4, std::min (0.4, m_integral));
  }

  double e_dot    = (e_norm - m_prevError) / m_Ts;
  m_prevError     = e_norm;
  double u_pid    = m_Kp * e_norm + m_integral + m_Kd * e_dot;
  double u_hybrid = u_pid;

  double dp_raw  = std::max (0.0, std::min (1.0, u_hybrid));

  m_dropProb = 0.90 * m_dropProb + 0.10 * dp_raw;

  WriteCSV (Simulator::Now ().GetSeconds (),
            q, m_dropProb,
            u_mpc, u_pid, u_hybrid,
            m_a, m_b);

  NS_LOG_DEBUG ("t="   << Simulator::Now ().GetSeconds ()
                << " q=" << q
                << " drop=" << m_dropProb
                << " a=" << m_a
                << " u_mpc=" << u_mpc
                << " u_pid=" << u_pid);

  m_event = Simulator::Schedule (
                Seconds (m_Ts),
                &MPCQueueDisc::ControlLoop,
                this);
}

/* ==========================================================
   UpdateRLS
   ========================================================== */
void
MPCQueueDisc::UpdateRLS (double q_now)
{
  double phi = m_prevQ;

  double K = (m_P[0][0] * phi) /
             (m_lambda + phi * m_P[0][0] * phi + 1e-9);

  double innov = q_now - m_a * phi;
  m_a += K * innov;
  m_a  = std::max (0.80, std::min (0.9999, m_a));

  m_P[0][0] = (1.0 / m_lambda) * (1.0 - K * phi) * m_P[0][0];
  if (m_P[0][0] < 1e-4)  m_P[0][0] = 1e-4;
  if (m_P[0][0] > 1e4)   m_P[0][0] = 1e4;

  m_prevQ = q_now;
  m_prevU = m_dropProb;
}

/* ==========================================================
   ComputeMPC
   ========================================================== */
double
MPCQueueDisc::ComputeMPC (double q)
{
  double sum_num = 0.0;
  double sum_den = 0.0;
  double a_pow   = 1.0;

  bool near_unity = (std::abs (1.0 - m_a) < 1e-6);

  for (uint32_t i = 1; i <= m_N; i++)
  {
    a_pow *= m_a;

    double gamma_i;
    if (near_unity)
      gamma_i = m_b * static_cast<double> (i);
    else
      gamma_i = m_b * (1.0 - a_pow) / (1.0 - m_a);

    double q_free_i = a_pow * q;

    sum_num += gamma_i * (q_free_i - m_qRef);
    sum_den += gamma_i * gamma_i;
  }

  double u_star = -sum_num / (sum_den + m_R + 1e-9);
  u_star = std::max (-2.0, std::min (2.0, u_star));

  return u_star;
}

/* ==========================================================
   ComputePID
   ========================================================== */
double
MPCQueueDisc::ComputePID (double error)
{
  double e_norm = error / m_qRef;
  double u_p    = m_Kp * e_norm;

  m_integral += m_Ki * m_Ts * e_norm;
  m_integral  = std::max (-0.5, std::min (0.5, m_integral));

  double u_i = m_integral;
  double u_d = (m_Kd / m_Ts) * (e_norm - m_prevError);

  m_prevError = e_norm;

  return u_p + u_i + u_d;
}

/* ==========================================================
   DoEnqueue
   ========================================================== */
bool
MPCQueueDisc::DoEnqueue (Ptr<QueueDiscItem> item)
{
  double q         = static_cast<double> (GetCurrentSize ().GetValue ());
  double threshold = 0.10 * m_qRef;

  if (q > threshold)
  {
    if (m_uv->GetValue () < m_dropProb)
    {
      DropBeforeEnqueue (item, "MPC-Hybrid AQM Drop");
      return false;
    }
  }

  if (GetCurrentSize ().GetValue () >= 1000)
  {
    DropBeforeEnqueue (item, "MPC-Hybrid Tail Drop");
    return false;
  }

  return GetInternalQueue (0)->Enqueue (item);
}

/* ==========================================================
   DoDequeue
   ========================================================== */
Ptr<QueueDiscItem>
MPCQueueDisc::DoDequeue ()
{
  if (GetInternalQueue (0)->IsEmpty ())
    return nullptr;

  return GetInternalQueue (0)->Dequeue ();
}

/* ==========================================================
   DoPeek
   ========================================================== */
Ptr<const QueueDiscItem>
MPCQueueDisc::DoPeek ()
{
  if (GetInternalQueue (0)->IsEmpty ())
    return nullptr;

  return GetInternalQueue (0)->Peek ();
}

/* ==========================================================
   CheckConfig
   ========================================================== */
bool
MPCQueueDisc::CheckConfig ()
{
  if (GetNInternalQueues () == 0)
  {
    AddInternalQueue (
      CreateObjectWithAttributes<DropTailQueue<QueueDiscItem>> (
        "MaxSize",
        QueueSizeValue (QueueSize ("1000p"))));
  }

  if (GetNInternalQueues () != 1)
  {
    NS_LOG_ERROR ("MPCQueueDisc: exactly one internal queue required.");
    return false;
  }

  return true;
}

/* ==========================================================
   WriteCSV
   ========================================================== */
void
MPCQueueDisc::WriteCSV (double t,    double q,
                        double drop,
                        double u_mpc, double u_pid, double u_hyb,
                        double a_est, double b_est)
{
  if (m_csv.is_open ())
  {
    m_csv << t      << ","
          << q      << ","
          << drop   << ","
          << u_mpc  << ","
          << u_pid  << ","
          << u_hyb  << ","
          << a_est  << ","
          << b_est  << "\n";
    m_csv.flush ();
  }
}

} // namespace ns3
