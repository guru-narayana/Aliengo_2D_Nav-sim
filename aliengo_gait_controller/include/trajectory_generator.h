#ifndef __TRAJECGEN_H__
#define __TRAJECGEN_H__

#include "ros/ros.h"
#include <vector>
#include <cmath>

using namespace std;
extern double T;
extern vector<vector<double>> blending_matrix;

vector<vector<double>> Robot_angular_motion_endpoints(double L1,double L,double W,double SL);
vector<double> Circle_intersection_points(double h,double k,double R1,double R2);
vector<vector<double>> Robot_end_points(vector<double> req_vel,  vector<vector<double>> mtn_angles, int swing, double SL,double H);


#endif