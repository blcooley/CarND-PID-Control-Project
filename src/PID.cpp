#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  gains[0] = Kp;
  gains[1] = Ki;
  gains[2] = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  cte_prev = 0.0;
  isWoundUpHigh = false;
  isWoundUpLow = false;
  shouldDebug = false;
}

void PID::UpdateError(double cte) {
  p_error = cte;

  // When controller is wound up (e.g., actuator at max), don't
  // continue to integrate error so response will be quick
  // when actuator needs to move from its limit.
  if ((cte > 0 && !isWoundUpHigh) || (cte < 0 && !isWoundUpLow)) {
    i_error += cte;
  } else {
    i_error = 0;
  }

  if (shouldDebug) {
    std::cout << "i_err: " << i_error << "   p_error: " << p_error << std::endl;
  }
  
  d_error = cte - cte_prev;
  cte_prev = cte;
}

double PID::TotalError() {
  // Use interactive form because it can be easier to tune
  return gains[0] * (p_error + gains[1] * i_error + gains[2] * d_error);
}

