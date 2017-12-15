//
//  interpolator.h
//  Path_Planning
//
//  Created by Srikanth Narayanan on 12/10/17.
//

#ifndef interpolator_h
#define interpolator_h

#include <iostream>
#include "spline.h"
#include <vector>

using namespace std;

/**
 A function that takes x, y, interval size and ouput size as inputs and uses the
 spline function to generate a smooth spline points
 */

vector<double> interpolator_func(vector<double> pts_x, vector<double> pts_y,
                                 double interval, int out_length)
{
  // check if x and y have equal lenght
  if(pts_x.size() != pts_y.size())
  {
    cout << "ERROR! : interpolator failed. X and Y should have same dimension" << endl;
    return {0};
  }
  
  tk::spline s;
  s.set_points(pts_x, pts_y);
  vector<double> output_y;
  for (int i = 0; i < out_length; i++)
  {
    output_y.push_back(s(pts_x[0] + i * interval));
  }
  return output_y;
}

/**
 OVERLOADED
 A function that takes x, y, interval size and ouput size as inputs and uses the
 spline function to generate a smooth spline points
 */

vector<double> interpolator_func(vector<double> pts_x, vector<double> pts_y,
                                 vector<double> eval_pts_x)
{
  // check if x and y have equal lenght
  if(pts_x.size() != pts_y.size())
  {
    cout << "ERROR! : interpolator failed. X and Y should have same dimension" << endl;
    return {0};
  }
  
  tk::spline s;
  s.set_points(pts_x, pts_y);
  vector<double> output_y;
  for (double x: eval_pts_x)
  {
    output_y.push_back(s(x));
  }
  return output_y;
}

#endif /* interpolator_h */
