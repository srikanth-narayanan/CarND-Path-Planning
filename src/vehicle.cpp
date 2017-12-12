//
//  vehicle.cpp
//  path_planning
//
//  Created by Srikanth Narayanan on 12/9/17.
//

#include <iostream>
#include <map>
#include <string>
#include "vehicle.h"
#include "constants.h"
#include "jmt.h"

using namespace std;

Vehicle::Vehicle() {}

Vehicle::Vehicle(double s, double s_dot, double s_ddot, double d, double d_dot, double d_ddot)
{
  this->s = s;
  this->s_dot = s_dot;
  this->s_ddot = s_ddot;
  this->d = d;
  this->d_dot = d_dot;
  this->d_ddot = d_ddot;
  this->current_state = "CS"; // CS - Constant Speed State is initial state before vehicle start
}

/*
 Destrcutor Definition
 */
Vehicle::~Vehicle() {}

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
    double future_s = this->s + s_dot * tm;
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

