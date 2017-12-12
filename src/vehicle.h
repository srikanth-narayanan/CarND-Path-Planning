//
//  vehicle.h
//  Path_Planning
//
//  Created by Srikanth Narayanan on 12/9/17.
//

#ifndef VEHICLE
#define VEHICLE

#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace std;

/**
 Class definition for vehicle objects
 */

class Vehicle
{
public:
  double s;
  double s_dot;
  double s_ddot;
  double d;
  double d_dot;
  double d_ddot;
  string current_state;
  vector<string> allowed_states;
  vector<double> s_traj_coeffs, d_traj_coeffs;
  
  /**
   Constructor
   */
  
  Vehicle();
  Vehicle(double s, double s_dot, double s_ddot, double d, double d_dot, double d_ddot);
  
  /**
   Destructor
   */
  virtual ~Vehicle();
  
  vector<vector<double>> generate_predictions(double start_time, double duration);
  
  void update_states(bool car_left, bool car_right);
};


#endif /* VEHICLE */
