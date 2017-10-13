#include "PID.h"
#include <cmath>
#include <iostream>
#include <fstream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki0, double ai, double Kd, bool twiddle) {
  p_error_ = 0;
  i_error_ = 0;
  d_error_ = 0;
  
  Kp_ = Kp;
  Ki0_ = Ki0;
  ai_ = ai;
  Ki_ = Ki0_;
  Kd_ = Kd;
  
  step_ = 1;
  twiddle_ = twiddle;
  twiddle_first_iteration_ = true;
  twiddle_settle_n_ = 100;
  twiddle_eval_n_ = 1000;
  twiddle_error_ = 0;
  twiddle_best_error_ = 10000000;
  twiddle_dp_[0] = 0.1 * Kp_;   // Tuning order P -> D -> I
  twiddle_dp_[1] = 0.1 * Kd_;
  twiddle_dp_[2] = 0.1 * Ki_;
  twiddle_param_index_ = 0;
  twiddle_tried_adding_ = false;
  twiddle_tried_subtracting_ = false;
  
  logfile_.open("../logs/log.txt", std::ios_base::app);
}

void PID::ParameterUpdate(double speed) {
  Ki_ = speed * ai_;
}

void PID::UpdateError(double cte) {
  // Exception for the first iteration
  if (step_ == 1)
  {
    p_error_ = cte;
  }

  // Update errors
  d_error_ = cte - p_error_;
  p_error_ = cte;
  i_error_ += cte;
  
  // Update twiddle error while running, allowing some settle time at every cycle
  // The idea is quite different from the one presented in the lesson, where the whole control was run for every twiddle cycle. Here I'm trying to do it in real time.
  if (step_ % (twiddle_settle_n_ + twiddle_eval_n_) > twiddle_settle_n_)
  {
    twiddle_error_ += cte*cte;
  }
  
  // Twiddle parameters just after having collected enough error measurements
  if (twiddle_ and (step_ % (twiddle_settle_n_ + twiddle_eval_n_) == 0))
  {
    cout << "Parameter twiddle" << endl;
    cout << "  step: " << step_ << endl;
    cout << "  error: " << twiddle_error_ << endl;
    cout << "  best error: " << twiddle_best_error_ << endl;
    
    if (twiddle_error_ < twiddle_best_error_)
    {
        cout << "  BETTER" << endl;
        twiddle_best_error_ = twiddle_error_;
        
        // Increase step, expect for the first time
        if (step_ !=  twiddle_settle_n_ + twiddle_eval_n_)
        {
            twiddle_dp_[twiddle_param_index_] *= 1.1;
        }
        
        // Move on to the next parameter and reset the state
        /*
        twiddle_param_index_ = (twiddle_param_index_ + 1) % 3;
        twiddle_tried_adding_ = false;
        twiddle_tried_subtracting_ = false;
        */
    }
    else
    {
      cout << "  WORSE" << endl;
    }
    
    cout << "  working on parameter: " << twiddle_param_index_ << endl;
    cout << "  dp: " << twiddle_dp_[0] << "  " << twiddle_dp_[1] << "  " << twiddle_dp_[2] << "  " << endl;
    cout << "  status: " << twiddle_tried_adding_ << " " << twiddle_tried_subtracting_ << endl;
    
    if (!twiddle_tried_adding_ and !twiddle_tried_subtracting_)
    {
        // Add dp[i]
        cout << "  +dp" << endl;
        Tune(twiddle_param_index_, twiddle_dp_[twiddle_param_index_]);
        twiddle_tried_adding_ = true;
    }
    else if (twiddle_tried_adding_ and !twiddle_tried_subtracting_)
    {
        // Subtract dp[i]
        cout << "  -2dp" << endl;
        Tune(twiddle_param_index_, -2.0 * twiddle_dp_[twiddle_param_index_]);
        twiddle_tried_subtracting_ = true;         
    }
    else
    {
        cout << "  +dp reset abd move on" << endl;
        // Set it back
        Tune(twiddle_param_index_, twiddle_dp_[twiddle_param_index_]);
        // Reduce dp[i]
        twiddle_dp_[twiddle_param_index_] *= 0.9;
        // Move on to the next parameter and reset the state
        twiddle_param_index_ = (twiddle_param_index_ + 1) % 3;
        twiddle_tried_adding_ = false;
        twiddle_tried_subtracting_ = false;
    }
    
    cout << "PDI new parameters: " << Kp_ << "    " << Kd_ << "    " << Ki_ << endl << endl;
    logfile_ << twiddle_error_ << ", " << twiddle_best_error_ << ", " << Kp_ << ", " << Ki_ << ", " << Kd_ << endl;
    
    // Reset the error accumulator
    twiddle_error_ = 0;
  }
  
  // Increase the step counter
  step_++;
}

double PID::TotalError() {
  double error = -Kp_ * p_error_
                 -Ki_ * i_error_
                 -Kd_ * d_error_;
  return error;
}

void PID::Tune(int index, double x) {
    if (index == 0) {
        Kp_ += x;
    }
    else if (index == 1) {
        Kd_ += x;
    }
    else if (index == 2) {
        Ki_ += x;
    }
    else {
    }
}

