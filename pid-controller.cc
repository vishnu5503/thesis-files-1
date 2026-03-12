#include "pid-controller.h"

namespace ns3 {

PIDController::PIDController(double Kp, double Ki, double Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd), prevError(0), integral(0) {}

double PIDController::GetControlSignal(double error) {
    integral += error;
    double derivative = error - prevError;
    prevError = error;
    return Kp * error + Ki * integral + Kd * derivative;
}

} // namespace ns3
