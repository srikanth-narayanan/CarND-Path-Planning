//
//  cost.h
//  Path_Planning
//
//  Created by Srikanth Narayanan on 12/7/17.
//

#ifndef COST
#define COST

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include "constants.h"

using namespace std;

//############################################################################//
//                                  Helpers                                   //
//############################################################################//

/**
A function that returns a value between 0 and 1 for x in the
range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
*/

double logistic(double x)
{
  return 2.0 / (1 + exp(-x)) - 1.0;
}

/**
 A function that calculates the nearest distance to an object from its s and d
 co-ordinates
 */
double nearest_approach(vector<double> s_traj, vector<double> d_traj, vector<vector<double>> prediction)
{
  double closest = 999999;
  for (int i = 0; i < N_SAMPLES; i++) {
    double dist = sqrt(pow(s_traj[i] - prediction[i][0], 2) + pow(d_traj[i] - prediction[i][1], 2));
    if (dist < closest) {
      closest = dist;
    }
  }
  return closest;
}

/**
 A function that calculates the minimum of all nearest distance to all objects.
 Prediction is a key, value mapped pair, where first value is key, sensor id and
 second value is predicted trajector s and d
 */
double nearest_approach_to_any_vehicle(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions)
{
  double closest = 999999;
  for (auto prediction : predictions) {
    double dist = nearest_approach(s_traj, d_traj, prediction.second);
    if (dist < closest) {
      closest = dist;
    }
  }
  return closest;
}

/**
 A function that calculates the minimum of all nearest distance to all objects
 in my own lane, becasue there could be multiple vehicle in own lane.
 Prediction is a key, value mapped pair, where first value is key, sensor id and
 second value is predicted trajector s and d
 */
double nearest_approach_to_any_vehicle_own_lane(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions)
{
  double closest = 999999;
  for (auto prediction : predictions) {
    double last_d = d_traj[d_traj.size() - 1];
    int my_lane = last_d / 4;
    vector<vector<double>> pred_traj = prediction.second;
    double pred_last_d = pred_traj[pred_traj.size() - 1][1];
    int pred_lane = pred_last_d / 4;
    if(my_lane == pred_lane)
    {
      double curr_dist = nearest_approach(s_traj, d_traj, prediction.second);
      if (curr_dist < closest && curr_dist < 120)
      {
        closest = curr_dist;
      }
    }
  }
  return closest;
}

/**
 A function to return derivatives of vectors with respect to time DT
 */

vector<double> differentiate(vector<double> quantity)
{
  // Given a trajectory, velocity can be returned, with velocity acceleration
  // and jerk etc., can be returned
  vector<double> derivative;
  for(int i = 1; i < quantity.size(); i++)
  {
    derivative.push_back((quantity[i] - quantity[i-1]) / DT);
  }
  return derivative;
}

//############################################################################//
//                               Cost Functions                               //
//############################################################################//

/**
 A function that Penalizes trajectories that span a duration which is longer or
 shorter than the duration requested.
 */
double time_diff_cost(double target_time, double actual_time)
{
  return logistic(fabs(actual_time - target_time) / target_time);
}

/**
 A function that Penalizes trajectories whose s coordinate (and derivatives)
 differ from the goal. Receive target s as s, s_dot and s_ddot
 */
double s_diff_cost(vector<double> s_traj, vector<double> target_s)
{
  int s_len = s_traj.size();
  double s1, s2, s3, s_dot1, s_dot2, s_ddot, cost = 0;
  s1 = s_traj[s_len - 1];
  s2 = s_traj[s_len - 2];
  s3 = s_traj[s_len - 3];
  s_dot1 = (s1 - s2) / DT;
  s_dot2 = (s2 - s3) / DT;
  s_ddot = (s_dot1 - s_dot2) / DT;
  cost += fabs(s1 - target_s[0]) / SIGMA_S;
  cost += fabs(s_dot1 - target_s[1]) / SIGMA_S_DOT;
  cost += fabs(s_ddot - target_s[2]) / SIGMA_S_DDOT;
  return logistic(cost);
}

/**
 A function that penalizes collision other vehicles.
 */
double collision_cost(vector<double> s_traj, vector<double> d_traj, map<int, vector<vector<double>>> predictions)
{
  double nearest_dist = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  if (nearest_dist < 2 * VEHICLE_RADIUS)
  {
    return 1.0;
  }
  else
  {
    return 0.0;
  }
}

/**
 A function that penalizes getting close to other vehicles
 */
double buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int, vector<vector<double>>> predictions)
{
  double nearest_dist = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest_dist);
}

/**
 A function that penalizes getting close to other vehicle in own lane
 */
double in_lane_buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int, vector<vector<double>>> predictions)
{
  double nearest = nearest_approach_to_any_vehicle_own_lane(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

/**
 A function that penalizes exceeding speed limits. If ds/ dt exceeds speed limit
 penalise the cost.
 */
double exceed_speed_limit_cost(vector<double> s_traj)
{
  vector<double> s_dot_traj = differentiate(s_traj);
  for(auto s_dot : s_dot_traj)
  {
    if(s_dot > SPEED_LIMIT)
    {
      return 1;
    }
  }
  return 0;
}

/**
 A function that rewards high average speeds
 */
double efficiency_cost(vector<double> s_traj)
{
  vector<double> s_dot_traj = differentiate(s_traj);
  double s_dot_latest = s_dot_traj[s_dot_traj.size() - 1];
  return logistic((SPEED_LIMIT - s_dot_latest) / SPEED_LIMIT);
}

/**
 A function that penalises exceeding max instaneous acceleration
 */
double max_accel_cost(vector<double> s_traj)
{
  vector<double> s_dot_traj = differentiate(s_traj);
  vector<double> s_ddot_traj = differentiate(s_dot_traj);
  for(auto s_ddot: s_ddot_traj)
  {
    if(s_ddot > MAX_INSTANTANEOUS_ACCEL)
    {
      return 1;
    }
  }
  return 0;
}

/**
 A function that penalises higher average acceleration
 */
double avg_accel_cost(vector<double> s_traj)
{
  vector<double> s_dot_traj = differentiate(s_traj);
  vector<double> s_ddot_traj = differentiate(s_dot_traj);
  double total_accel = 0.0;
  for(auto s_ddot: s_ddot_traj)
  {
    total_accel += s_ddot;
  }
  double avg_accel = total_accel / s_ddot_traj.size();
  return logistic(avg_accel / EXPECTED_ACC_IN_ONE_SEC);
}

/**
 A function that penalises exceeding max instaneous jerk
 */
double max_jerk_cost(vector<double> s_traj)
{
  vector<double> s_dot_traj = differentiate(s_traj);
  vector<double> s_ddot_traj = differentiate(s_dot_traj);
  vector<double> s_dddot_traj = differentiate(s_ddot_traj);
  for(auto s_dddot: s_dddot_traj)
  {
    if(s_dddot > MAX_INSTANTANEOUS_JERK)
    {
      return 1;
    }
  }
  return 0;
}

/**
 A function that penalises higher average jerk
 */
double avg_jerk_cost(vector<double> s_traj)
{
  vector<double> s_dot_traj = differentiate(s_traj);
  vector<double> s_ddot_traj = differentiate(s_dot_traj);
  vector<double> s_dddot_traj = differentiate(s_ddot_traj);
  double total_jerk = 0.0;
  for(auto s_dddot: s_dddot_traj)
  {
    total_jerk += s_dddot;
  }
  double avg_jerk = total_jerk / s_dddot_traj.size();
  return logistic(avg_jerk / EXPECTED_JERK_IN_ONE_SEC);
}

/**
 A function that penalises not trying to be in middle lane.
 */
double not_maintaining_middle(vector<double> d_traj)
{
  double last_d = d_traj[d_traj.size() - 1];
  double delta_sqr_d = std::pow((last_d - 6), 2);
  return logistic(delta_sqr_d); // d =6 is the middle lane
}

/**
 A function that calculates the total cost
 */
double calc_total_cost(vector<double> s_traj, vector<double> d_traj, map<int, vector<vector<double>>> predictions)
{
  double total_cost = 0.0;
  double coll_cost = collision_cost(s_traj, d_traj, predictions);
  double buf_cost = buffer_cost(s_traj, d_traj, predictions);
  double in_lane_cost = in_lane_buffer_cost(s_traj, d_traj, predictions);
  double eff_cost = efficiency_cost(s_traj);
  double not_mid_cost = not_maintaining_middle(d_traj);
  
  total_cost += coll_cost + buf_cost + in_lane_cost + eff_cost + not_mid_cost;
  
  return total_cost;
}

#endif /* cost */
