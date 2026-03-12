#ifndef MPC_QUEUE_DISC_H
#define MPC_QUEUE_DISC_H

/**
 * =======================================================
 *  Hybrid RLS-MPC + PID Adaptive Queue Discipline
 * =======================================================
 *
 * PLANT MODEL (2-parameter, 1st-order discrete-time):
 *   q[k] = a*q[k-1] + b*u[k-1]
 *   where q = queue length (packets)
 *         u = drop probability in [0,1]
 *
 * IDENTIFICATION — RLS (Recursive Least Squares):
 *   Parameter vector  : theta = [a, b]^T
 *   Regression vector : phi   = [q[k-1], u[k-1]]^T
 *   Kalman gain       : K = P*phi / (lambda + phi^T*P*phi)
 *   Parameter update  : theta += K * (q[k] - phi^T*theta)
 *   Covariance update : P = (1/lambda)*(I - K*phi^T)*P
 *   Forgetting factor : lambda in (0,1) — tracks slow variation
 *
 * CONTROL — MPC (Model Predictive Control):
 *   Move-blocking: single u applied over full horizon N.
 *   N-step prediction: q_pred[i] = a^i * q + gamma_i * u
 *     where gamma_i = b*(1 - a^i)/(1 - a)  [sum of geometric series]
 *   Quadratic cost:
 *     J = sum_{i=1}^{N} (q_pred[i] - qRef)^2 + R*u^2
 *   Analytical optimum (dJ/du = 0):
 *     u* = -[sum_i gamma_i*(a^i*q - qRef)] / [sum_i gamma_i^2 + R]
 *
 * CONTROL — PID (discrete-time parallel form):
 *   error = q[k] - qRef
 *   u_p   = Kp * error
 *   u_i  += Ki * Ts * error          (clamped anti-windup)
 *   u_d   = (Kd/Ts) * (error - prevError)
 *   u_pid = u_p + u_i + u_d
 *
 * HYBRID COMBINATION:
 *   delta = Km * u_mpc + Wpid * u_pid
 *   dropProb = clamp[0,1]( dropProb + delta )
 *
 * SCHEDULING:
 *   ControlLoop() fires every Ts seconds (NS-3 scheduler).
 *   DoEnqueue()   applies probabilistic drop using dropProb.
 * =======================================================
 */

#include "ns3/queue-disc.h"
#include "ns3/event-id.h"
#include "ns3/random-variable-stream.h"
#include <fstream>

namespace ns3 {

class MPCQueueDisc : public QueueDisc
{
public:
  static TypeId GetTypeId (void);
  MPCQueueDisc  ();
  virtual ~MPCQueueDisc ();

  // ── Runtime control interface (for mid-simulation changes) ────────────
  /**
   * SetQueueRef: change the target queue length at any simulation time.
   * Performs a bumpless transfer by resetting the integral accumulator
   * and derivative memory so the controller does not lurch from
   * windup that was tuned for the previous reference.
   */
  void   SetQueueRef (double newRef);

  /** GetQueueRef: read the currently active queue reference. */
  double GetQueueRef () const;

  /** GetDropProb: read the current drop probability (0..1). */
  double GetDropProb () const;

private:
  /* ── Required QueueDisc interface ──────────────────── */
  bool                     DoEnqueue       (Ptr<QueueDiscItem> item) override;
  Ptr<QueueDiscItem>       DoDequeue       ()                        override;
  Ptr<const QueueDiscItem> DoPeek          ()                        override;
  bool                     CheckConfig     ()                        override;
  void                     InitializeParams()                        override;

  /* ── Periodic control loop (scheduled every m_Ts s) ── */
  void   ControlLoop ();

  /* ── Sub-routines called inside ControlLoop ─────────── */
  void   UpdateRLS   (double q_now);          // RLS identification
  double ComputeMPC  (double q);              // MPC optimal u*
  double ComputePID  (double error);          // PID output

  /* ── Logging ─────────────────────────────────────────── */
  void   WriteCSV    (double t, double q,
                      double dropProb,
                      double u_mpc, double u_pid, double u_hyb,
                      double a_est, double b_est);

  /* ============================================================
     MEMBER VARIABLES
     ============================================================ */

  /* --- Tunable attributes (exposed via NS-3 Attributes) --- */
  double   m_qRef;     // target queue length (packets)
  double   m_Ts;       // sampling period (s), default 0.01
  uint32_t m_N;        // MPC prediction horizon
  double   m_R;        // MPC control effort weight
  double   m_Km;       // MPC contribution gain
  double   m_Wpid;     // PID contribution weight
  double   m_Kp;       // PID proportional gain
  double   m_Ki;       // PID integral gain
  double   m_Kd;       // PID derivative gain
  double   m_lambda;   // RLS forgetting factor

  /* --- RLS state --- */
  double m_a;          // identified plant pole  (initialised 0.95)
  double m_b;          // identified plant gain  (initialised -0.002)
  double m_P[2][2];    // 2x2 RLS covariance matrix
  double m_prevQ;      // q[k-1]  — RLS regressor element 0
  double m_prevU;      // u[k-1]  — RLS regressor element 1

  /* --- PID state --- */
  double m_integral;   // integral accumulator I[k]
  double m_prevError;  // e[k-1]  — for derivative term

  /* --- Output --- */
  double m_dropProb;   // current drop probability, in [0, 1]

  /* --- NS-3 internals --- */
  EventId                    m_event;  // handle for scheduled ControlLoop
  Ptr<UniformRandomVariable> m_uv;     // U[0,1] for probabilistic drop

  /* --- Output file --- */
  std::ofstream m_csv;
};

} // namespace ns3

#endif // MPC_QUEUE_DISC_H
