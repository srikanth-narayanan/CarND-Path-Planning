//
//  jmt.cpp
//  path_planning
//
//  Created by Srikanth Narayanan on 12/9/17.
//

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "jmt.h"


using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;


vector<double> get_jmt_coeffs(vector<double> start, vector <double> end, double T)
{
  /*
   Calculate the Jerk Minimizing Trajectory that connects the initial state
   to the final state in time T.
   
   INPUTS
   
   start - the vehicles start location given as a length three array
   corresponding to initial values of [s, s_dot, s_double_dot]
   
   end   - the desired end state for vehicle. Like "start" this is a
   length three array.
   
   T     - The duration, in seconds, over which this maneuver should occur.
   
   OUTPUT
   an array of length 6, each value corresponding to a coefficent in the polynomial
   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   
   EXAMPLE
   
   > JMT( [0, 10, 0], [10, 10, 0], 1)
   [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T,  T*T*T*T*T,
       3*T*T, 4*T*T*T,  5*T*T*T*T,
       6*T,   12*T*T,   20*T*T*T;
  
  MatrixXd Ainv = A.inverse();
  
  MatrixXd B = MatrixXd(3,1);
  B << end[0] - (start[0] + start[1]*T + .5*start[2]*T*T),
       end[1] - (start[1] + start[2]*T),
       end[2] - start[2];
  
  MatrixXd C = Ainv * B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
    result.push_back(C.data()[i]);
  }
  return result;
}
