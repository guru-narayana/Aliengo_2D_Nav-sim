#include "Kinematics.h"

double L1 = 0.083;
double L2 = 0.25;
double L3 = 0.25;
double Length = 0.2399*2;
double width = 0.051*2;


vector<double> Left_Leg_IK(vector<double> xyz_state){
    double P1,P2,P3,P4;
    vector<double> thta = {0,0,0};

    P1 = sqrt( pow(xyz_state[0],2) + pow(xyz_state[1],2) - pow(L1,2));
    P2 = sqrt(pow(P1,2) + pow(xyz_state[2],2));
    P3 = (pow(P2,2)+pow(L2,2)-pow(L3,2))/(2*P2*L2);
    P4 = -(pow(L2,2)+pow(L3,2)-pow(P2,2))/(2*L3*L2);
    thta[0] = atan(P1/L1) + atan(xyz_state[1]/xyz_state[0]) - M_PI_2; // theta_1 calculation
    thta[1] = atan(-xyz_state[2]/P1) + atan(sqrt(1-pow(P3,2))/P3);    // theta_2 calculation
    thta[2] = -acos(P4);                            // theta_3 calculation
    return thta;
}

vector<double> Right_Leg_IK(vector<double> xyz_state){
    double P1,P2,P3,P4;
    vector<double> thta = {0,0,0};

    P1 = sqrt( pow(xyz_state[0],2) + pow(xyz_state[1],2) - pow(L1,2));
    P2 = sqrt(pow(P1,2) + pow(xyz_state[2],2));
    P3 = (pow(P2,2)+pow(L2,2)-pow(L3,2))/(2*P2*L2);
    P4 = -(pow(L2,2)+pow(L3,2)-pow(P2,2))/(2*L3*L2);

    thta[0] = atan(P1/L1) + atan(-xyz_state[1]/xyz_state[0]) - M_PI_2;   // theta_1 calculation
    thta[1] = -(atan(-xyz_state[2]/P1) + atan(sqrt(1-pow(P3,2))/P3));    // theta_2 calculation
    thta[2] = acos(P4);                                // theta_3 calculation
    return thta;
}

vector<double> Left_Leg_FK(vector<double> cjs){                        //current joint state - cjs
    vector<double> xyz_state = {0,0,0};
    xyz_state[0] = L3*cos(cjs[0])*cos(cjs[1]+cjs[2]) + L2*cos(cjs[0])*cos(cjs[1]) - L1*sin(cjs[0]);
    xyz_state[1] = L3*sin(cjs[0])*cos(cjs[1]+cjs[2]) + L2*sin(cjs[0])*cos(cjs[1]) + L1*cos(cjs[0]);
    xyz_state[2] = -L3*sin(cjs[1]+cjs[2]) - L2*sin(cjs[1]);
    return xyz_state;
}
vector<double> Right_Leg_FK(vector<double> cjs){                         //current joint state - cjs
    vector<double> xyz_state = {0,0,0};
    xyz_state[0] = L3*cos(cjs[0])*cos(cjs[1]+cjs[2]) + L2*cos(cjs[0])*cos(cjs[1]) + L1*sin(cjs[0]);
    xyz_state[1] = L3*sin(cjs[0])*cos(cjs[1]+cjs[2]) + L2*sin(cjs[0])*cos(cjs[1]) - L1*cos(cjs[0]);
    xyz_state[2] = L3*sin(cjs[1]+cjs[2]) + L2*sin(cjs[1]);
    return xyz_state;
}