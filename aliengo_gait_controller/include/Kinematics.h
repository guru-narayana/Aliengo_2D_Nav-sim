#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "ros/ros.h"
#include <vector>
#include <cmath>

using namespace std;

extern double L1;
extern double L2;
extern double L3;
extern double Length;
extern double width;

vector<double> Left_Leg_IK(vector<double> xyz_state);
vector<double> Right_Leg_IK(vector<double> xyz_state);
vector<double> Left_Leg_FK(vector<double> cjs);
vector<double> Right_Leg_FK(vector<double> cjs);

#endif