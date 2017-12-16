//
//  vehicle.cpp
//  path_planning
//
//  Created by Srikanth Narayanan on 12/9/17.
//

#include <iostream>
#include <map>
#include <string>
#include <algorithm>
#include <cmath>
#include "vehicle.h"
#include "constants.h"
#include "jmt.h"

using namespace std;
/*
 Constructor Definition
 */
Vehicle::Vehicle() {}

/*
 Destrcutor Definition
 */
Vehicle::~Vehicle() {}

void Vehicle::Init(double s, double s_dot, double s_ddot, double d, double d_dot, double d_ddot)
{
  this->s = s;
  this->s_dot = s_dot;
  this->s_ddot = s_ddot;
  this->d = d;
  this->d_dot = d_dot;
  this->d_ddot = d_ddot;
  state = "CS"; // CS - Constant Speed State is initial state before vehicle start
}

/**
 A vehicle class method to generate a trajectory of predicted s and d for other
 vehicles on lanes. Since ego car trajector is based on previous position, other
 object should also have trajectory start from same time
 */
vector<vector<double>> Vehicle::generate_predictions(double start_time, double duration)
{
  vector<vector<double>> predictions;
  for(int i = 0; i < N_SAMPLES; i++)
  {
    double tm = start_time + (i * duration/N_SAMPLES);
    double future_s = this->s + this->s_dot * tm;
    vector<double> s_d = {future_s, this->d};
    predictions.push_back(s_d);
  }
  return predictions;
}

/**
 The vehicle has a total of 4 states.
 CS - Constant Speed - Initial State
 KL - Keep Lane - Vehicle reach target speed unless there is an object in front.
 Slows down to vehicle in front like Adaptive Cruise Control.
 LCL or LCR - lane change left or lane change right will perform a lane change
 maneveure and transit to KL state.
 
 A class method to update the possible states beased on objects
 */
void Vehicle::update_states(bool car_left, bool car_right)
{
  this->allowed_states = {"KL"};
  if(this->d > 4 && !car_left)
  {
    this->allowed_states.push_back("LCL");
  }
  if(this->d < 8 && !car_right)
  {
    this->allowed_states.push_back("LCR");
  }
}

/**
 This class method is a helper that generates the s, s_dot, s_ddot, d, d_dot, d_ddot
 for a given situation of the own car. The method returns a set of s and d values.
 The speed control of the car is based on frenet trajectory and whether a car is
 present in front of the Ego car or not. If car is in front the Ego car s_dot is
 set to same as car in front s_dot from predictions otherwise it will reach the
 s_dot based on target velocity in own lane.
 */

vector<vector<double>> Vehicle::_get_target_on_state(string state, map<int, vector<vector<double>>> predictions, double duration, bool car_front)
{
  int target_lane, ego_lane = this->d / 4;
  double d;
  double d_dot = 0;
  double d_ddot = 0;
  double s_dot = min(this->s_dot + MAX_INSTANTANEOUS_ACCEL/4 * duration, SPEED_LIMIT);
  s_dot = SPEED_LIMIT;
  double s_ddot = 0;
  
  // Displacement = Ego displacement + Average Velocity * duration
  double s = this->s + (this->s_dot + s_dot)/2 * duration;
  
  if(state.compare("KL") == 0)
  {
    d = (double)ego_lane * 4 + 2;
    target_lane = d / 4;
  }
  else if(state.compare("LCL") == 0)
  {
    d = ((double)ego_lane - 1) * 4 + 2;
    target_lane = d / 4;
  }
  else if(state.compare("LCR") == 0)
  {
    d = ((double)ego_lane + 1) * 4 + 2;
    target_lane = d / 4;
  }
  
  // If there is a vehicle in front
  vector<double> object_s_d = _get_object_data(target_lane, predictions, duration);
  double object_s = object_s_d[0];
  double object_s_dot = object_s_d[1];
  if(object_s - s < FOLLOWING_DISTANCE && object_s > this->s)
  {
    s_dot = object_s_dot;
    
    if(fabs(object_s - s) < 0.5 * FOLLOWING_DISTANCE)
    {
      s_dot -= 1; // Slow down
    }
    s = object_s - FOLLOWING_DISTANCE;
  }
  // Slow down rapidly
  if(car_front)
  {
    s_dot = 0.0;
  }
  
  // Do i need also an emergency brake ? What is the point w.r.t this project
  return {{s, s_dot, s_ddot},{d, d_dot, d_ddot}};
}

/**
 This class method return the nearest object s and s_dot for the object in front.
 This is based on the assumption that the Ego vehicle lane and velocity remains
 the same.
 */
vector<double> Vehicle::_get_object_data(int target_lane, map<int, vector<vector<double>>> predictions, double duration)
{
  double object_s_dot = 0;
  double object_s = 99999;
  
  for(auto predict : predictions)
  {
    vector<vector<double>> predict_trajectory = predict.second;
    int obj_lane = predict_trajectory[0][1] / 4;
    if(obj_lane == target_lane)
    {
      double s0 = predict_trajectory[0][0];
      double sn = predict_trajectory[predict_trajectory.size() - 1][0];
      double sn_1 = predict_trajectory[predict_trajectory.size() - 2][0];
      double delta_t = duration / N_SAMPLES;
      double s_dot = (sn - sn_1) / delta_t;
      if (sn < object_s && s0 > this->s)
      {
        object_s = sn;
        object_s_dot = s_dot;
      }
    }
  }
  return {object_s, object_s_dot};
}

/**
 This class method generates a Jerk Minimised trajectory for a given target and
 trajectory duration.
 */
vector<vector<double>> Vehicle::generate_trajectory(vector<vector<double>> target_s_d, double duration)
{
  vector<double> end_s = target_s_d[0];
  vector<double> end_d = target_s_d[1];
  vector<double> start_s = {this->s, this->s_dot, this->s_ddot};
  vector<double> start_d = {this->d, this->d_dot, this->d_ddot};
  
  // calculate JMT
  this->s_traj_coeffs = get_jmt_coeffs(start_s, end_s, duration);
  this->d_traj_coeffs = get_jmt_coeffs(start_d, end_d, duration);
  
  // calcuate traj from coeff
  vector<double> s_traj;
  vector<double> d_traj;
  
  for(int i = 0; i < N_SAMPLES; i++)
  {
    double delta_t = i * duration/N_SAMPLES;
    double s = 0;
    double d = 0;
    for(int j = 0; j < s_traj_coeffs.size(); j++)
    {
      s += this->s_traj_coeffs[j] * pow(delta_t, j);
      d += this->d_traj_coeffs[j] * pow(delta_t, j);
    }
    s_traj.push_back(s);
    d_traj.push_back(d);
  }
  return {s_traj, d_traj};
}


