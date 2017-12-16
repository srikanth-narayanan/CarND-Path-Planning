//
//  constants.h
//  Path_Planning
//
//  Created by Srikanth Narayanan on 12/7/17.
//

#ifndef CONSTANTS
#define CONSTANTS

#define TRACK_LENGTH 6945.554   // Total S distance of the circuit
#define VEHICLE_RADIUS 1.25     // A safe radius bubble around the vehicle in meters
#define FOLLOWING_DISTANCE 8.0  // meters in frenet s
#define POINTS_IN_SPLINE 50     // points the final trajectory is made. smooth
#define PATH_DT 0.02            // Path distance intreval in seconds
#define POINTS_FROM_PREVIOUS 25 // number of points to keep from previous path

#define EXPECTED_MAX_JERK 2     // m/s3
#define EXPECTED_MAX_ACCEL 1    // m/s2

#define SPEED_LIMIT 21.25          // m/s
#define VELOCITY_INCRE_LIMIT  0.125

#define INTERPOLATION_AHEAD 5  // Number of points
#define INTERPOLATION_BEHIND 5 // Number of points
#define POINTS_INCR 0.5 // each point on the spline is split by 0.5 meters in Frenet s

// sigma values for perturbing targets
#define SIGMA_S 10.0            // s
#define SIGMA_S_DOT 3.0         // s_dot
#define SIGMA_S_DDOT 0.1        // s
#define SIGMA_D 0               // d
#define SIGMA_D_DOT 0           // d_dot
#define SIGMA_D_DDOT 0          // d_double_dot
#define SIGMA_T 0.05            // T

// weights of costs
#define COLLISION_COST 99999       // Cost of colliding with another vehicle
#define BUFFER_COST 10              // Cost of maintaining buffer with vehicles in other lane
#define IN_LANE_BUFFER_COST 1000    // Cost of maintaining buffer with vehicle in own lane
#define EFFICIENCY_COST 10000       // Cost of overall efficiency
#define MIDDLE_LANE_COST 100

#define MAX_INSTANTANEOUS_JERK 10   // Cost of maximum instaneous Jerk
#define MAX_INSTANTANEOUS_ACCEL 10  // Cost of maximum instaneous accel

#define EXPECTED_JERK_IN_ONE_SEC 2  // m/s3
#define EXPECTED_ACC_IN_ONE_SEC 1   // m/s2

// Trajectory Length
#define N_SAMPLES 20
#define DT 0.20                 // Cycle time

#endif /* CONSTANTS */
