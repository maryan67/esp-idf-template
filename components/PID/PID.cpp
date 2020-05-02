#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <cmath>
#include "pid.h"
#include <stdio.h>

using namespace std;

class PIDImpl
{
public:
    PIDImpl(double max, double min, double Kp, double Kd, double Ki);
    ~PIDImpl();
    double calculate(double setpoint, double pv, double _dt);

private:
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};

PID::PID(double max, double min, double Kp, double Kd, double Ki)
{
    pimpl = new PIDImpl(max, min, Kp, Kd, Ki);
}
double PID::calculate(double setpoint, double pv, double _dt)
{
    return pimpl->calculate(setpoint, pv, _dt);
}
PID::~PID()
{
    delete pimpl;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl(double max, double min, double Kp, double Kd, double Ki) : _max(max),
                                                                            _min(min),
                                                                            _Kp(Kp),
                                                                            _Kd(Kd),
                                                                            _Ki(Ki),
                                                                            _pre_error(0),
                                                                            _integral(0)
{
}

double PIDImpl::calculate(double setpoint, double pv, double _dt)
{

    // Calculate error
    double error = -1*(setpoint - pv >= 0 ? pv : -1 * pv);

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt / 1000;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt / 1000;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif