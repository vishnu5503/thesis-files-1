#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

namespace ns3 {

class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd);
    double GetControlSignal(double error);

private:
    double Kp, Ki, Kd;
    double prevError, integral;
};

} // namespace ns3

#endif // PID_CONTROLLER_H
