//
//  vehicle.h
//  Path_Planning
//
//  Created by Srikanth Narayanan on 12/9/17.
//

#ifndef VEHICLE
#define VEHICLE

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
  string state;
  vector<string> allowed_states;
  vector<double> s_traj_coeffs, d_traj_coeffs;
  
  /**
   Constructor
   */
  Vehicle();
  
  /**
   Destructor
   */
  virtual ~Vehicle();
  
  void Init(double s, double s_dot, double s_ddot, double d, double d_dot, double d_ddot);
  
  vector<vector<double>> generate_predictions(double start_time, double duration);
  
  void update_states(bool car_left, bool car_right);
  
  vector<vector<double>> _get_target_on_state(string state, map<int, vector<vector<double>>> predictions, double duration, bool car_front);
  
  vector<double> _get_object_data(int target_lane, map<int, vector<vector<double>>> predictions, double duration);
  
  vector<vector<double>> generate_trajectory(vector<vector<double>> target_s_d, double duration);
};


#endif /* VEHICLE */
