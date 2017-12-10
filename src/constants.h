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

#define MAX_JERK 10             // m/s3
#define MAX_ACCEL 10            // m/s2

#define EXPECTED_MAX_JERK 2     // m/s3
#define EXPECTED_MAX_ACCEL 1    // m/s2

#define SPEED_LIMIT 22          // m/s
#define VELOCITY_INCRE_LIMIT  0.225

// sigma values for perturbing targets
#define SIGMA_S 10.0            // s
#define SIGMA_S_DOT 3.0         // s_dot
#define SIGMA_S_DDOT 0.1        // s
#define SIGMA_D 0               // d
#define SIGMA_D_DOT 0           // d_dot
#define SIGMA_D_DDOT 0          // d_double_dot
#define SIGMA_T 0.05            // T
#define DT 0.02                 // Cycle time

// weights of costs
#define COLLISION_COST 999999       // Cost of colliding with another vehicle
#define BUFFER_COST 10              // Cost of maintaining buffer with vehicles in other lane
#define IN_LANE_BUFFER_COST 1000    // Cost of maintaining buffer with vehicle in own lane
#define EFFICIENCY_COST 10000       // Cost of overall efficiency
#define MAX_INSTANTANEOUS_JERK 10   // Cost of maximum instaneous Jerk
#define MAX_INSTANTANEOUS_ACCEL 10  // Cost of maximum instaneous accel
#define EXPECTED_JERK_IN_ONE_SEC 2  // m/s3
#define EXPECTED_ACC_IN_ONE_SEC 1   // m/s2

// Trajectory Length
#define N_SAMPLES 20

#endif /* CONSTANTS */
