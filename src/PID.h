#ifndef PID_H
#define PID_H

#include <fstream>

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;
  
  /*
  * Coefficient linking speed and PID coefficients
  */
  double ai_ = 0.001;
  
  /*
  * Twiddle
  */
  unsigned int step_;
  bool twiddle_;
  bool twiddle_first_iteration_;
  unsigned int twiddle_settle_n_;
  unsigned int twiddle_eval_n_;
  double twiddle_error_;
  double twiddle_best_error_;
  double twiddle_dp_[3];
  unsigned short twiddle_param_index_;
  bool twiddle_tried_adding_, twiddle_tried_subtracting_;
  
  std::ofstream logfile_;
  
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double ai_, bool twiddle);
  
  /*
  * Parameter update based on car speed
  */
  void ParameterUpdate(double speed);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * Tune specific PID parameter.
  */
  void Tune(int index, double x);
};

#endif /* PID_H */
