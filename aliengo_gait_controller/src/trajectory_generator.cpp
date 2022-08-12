#include "trajectory_generator.h"
#include "Kinematics.h"

double T = 0; // Time Period of the trajectory
vector<vector<double>> blending_matrix ={{1,-3,2},
                                         {0,4,-4},
                                         {0,-1,2}};

vector<double> Circle_intersection_points(double h,double k,double R1,double R2){
    vector<double> intersection_pnts = {0,0,0,0};
    intersection_pnts[0] = ((pow(R1,2)*h - pow(R2,2)*h + h*pow(k,2) + k*sqrt((-pow((R1-R2),2)+pow(h,2)+pow(k,2))*(pow((R1+R2),2)-pow(h,2)-pow(k,2))) + pow(h,3))/(2*(pow(h,2)+pow(k,2))))-h;
    intersection_pnts[1] = ((pow(R1,2)*h - pow(R2,2)*h + h*pow(k,2) - k*sqrt((-pow((R1-R2),2)+pow(h,2)+pow(k,2))*(pow((R1+R2),2)-pow(h,2)-pow(k,2))) + pow(h,3))/(2*(pow(h,2)+pow(k,2))))-h;
    intersection_pnts[2] = ((pow(R1,2)*k - pow(R2,2)*k + k*pow(h,2) + h*sqrt((-pow((R1-R2),2)+pow(h,2)+pow(k,2))*(pow((R1+R2),2)-pow(h,2)-pow(k,2))) + pow(k,3))/(2*(pow(h,2)+pow(k,2))))-k;
    intersection_pnts[3] = ((pow(R1,2)*k - pow(R2,2)*k + k*pow(h,2) - h*sqrt((-pow((R1-R2),2)+pow(h,2)+pow(k,2))*(pow((R1+R2),2)-pow(h,2)-pow(k,2))) + pow(k,3))/(2*(pow(h,2)+pow(k,2))))-k;
    return intersection_pnts;
}

vector<vector<double>> Robot_angular_motion_endpoints(double L1,double L,double W,double SL){ // Link1 ,Length , Width and steplength
    // This function  returns the angle in which each foot should move inorder to achive rotational motion with respect to center of the robot
    vector<double> right_turn_angles = {0,0,0,0};
    vector<double> left_turn_angles = {0,0,0,0};
    double Rr = sqrt( pow((L1+W/2),2)  + pow(L/2,2));
    double h{},k{};
    // For front Left
    h = W/2+L1;
    k = L/2;
    vector<double> FL_intersections = Circle_intersection_points(h,k,Rr,SL/2);
    right_turn_angles[0] = atan2(FL_intersections[2],FL_intersections[1]);
    left_turn_angles[0] = atan2(FL_intersections[3],FL_intersections[0]);
        //cout<<FL_intersections[0]<<"  "<<FL_intersections[1]<<"  "<<FL_intersections[2]<<"  "<<FL_intersections[3]<<"\n";
    // For Front Right
    h = -W/2-L1;
    k = L/2;
    vector<double> FR_intersections = Circle_intersection_points(h,k,Rr,SL/2);
    right_turn_angles[1] = atan2(FR_intersections[2],FR_intersections[1]);
    left_turn_angles[1] = atan2(FR_intersections[3],FR_intersections[0]);
        //cout<<FR_intersections[0]<<"  "<<FR_intersections[1]<<"  "<<FR_intersections[2]<<"  "<<FR_intersections[3]<<"\n";
    // For Rear Left
    h = W/2+L1;
    k = -L/2;
    vector<double> RL_intersections = Circle_intersection_points(h,k,Rr,SL/2);
    right_turn_angles[2] = atan2(RL_intersections[2],RL_intersections[1]);
    left_turn_angles[2] = atan2(RL_intersections[3],RL_intersections[0]);
        //cout<<RL_intersections[0]<<"  "<<RL_intersections[1]<<"  "<<RL_intersections[2]<<"  "<<RL_intersections[3]<<"\n";
    // For Rear Right
    h = -W/2-L1;
    k = -L/2;
    vector<double> RR_intersections = Circle_intersection_points(h,k,Rr,SL/2);
    right_turn_angles[3] = atan2(RR_intersections[2],RR_intersections[1]);
    left_turn_angles[3] = atan2(RR_intersections[3],RR_intersections[0]);
        //cout<<RR_intersections[0]<<"  "<<RR_intersections[1]<<"  "<<RR_intersections[2]<<"  "<<RR_intersections[3]<<"\n";
    vector<vector<double>> output_angles = {right_turn_angles ,left_turn_angles};
    return output_angles;
}

vector<vector<double>> Robot_end_points(vector<double> req_vel,  vector<vector<double>> mtn_angles, int swing, double SL,double H){
    double z_offset = 0.03;

    if(req_vel[0] ==0 && req_vel[1] == 0){
        vector<double> FL_end_point = {H,L1,z_offset};
        vector<double> FR_end_point = {H,-L1,z_offset};
        vector<double> RL_end_point = {H,L1,z_offset};
        vector<double> RR_end_point = {H,-L1,z_offset};
        vector<vector<double>> out = {FL_end_point,FR_end_point,RL_end_point,RR_end_point};
        T = 0.1;
        return out;
    }



    
    double Rr = sqrt( pow((L1+width/2),2)  + pow(Length/2,2));
    double x = 1 - pow(SL,2)/(2*pow(Rr,2));
    double theta = atan(sqrt(1-pow(x,2))/x);  
    double angular_velocity = abs((req_vel[1]*SL)/theta);
    double velocity{};
    // For Front Left
    vector<double> FL_end_point = {0,0,0};
    double FL_theta{};
    if(swing){
        if(req_vel[1]>0) FL_theta = mtn_angles[1][0];
        else FL_theta = mtn_angles[0][0];

    }
    else{
        if(req_vel[1]>0) FL_theta = mtn_angles[0][0];
        else FL_theta = mtn_angles[1][0];   
    }
    // finding the effective velocity
    double y = angular_velocity*cos(FL_theta);
    double z{};
    if(swing) z = angular_velocity*sin(FL_theta) + req_vel[0];
    else z = angular_velocity*sin(FL_theta) - req_vel[0];
    FL_theta = atan2(z,y);
    FL_end_point[0] = H;
    FL_end_point[1] = cos(FL_theta)*SL/2 + L1; // L1 is the offset aas coordinate system for calculation it is considered at end of L1 not at the beggining 
    FL_end_point[2] = sin(FL_theta)*SL/2+ z_offset;
    //cout<<FL_end_point[0]<<"   "<<FL_end_point[1]<<"   "<<FL_end_point[2]<<"     "<<swing<<"\n";


    // For Rear Right
    vector<double> RR_end_point = {0,0,0};
    double RR_theta{};
    if(swing){
        if(req_vel[1]>0) RR_theta = mtn_angles[1][3];
        else RR_theta = mtn_angles[0][3];
    }
    else{
        if(req_vel[1]>0) RR_theta = mtn_angles[0][3];
        else RR_theta = mtn_angles[1][3];   
    }
    y = angular_velocity*cos(RR_theta);
    if(swing) z = angular_velocity*sin(RR_theta) + req_vel[0];
    else z = angular_velocity*sin(RR_theta) - req_vel[0];
    RR_theta = atan2(z,y);
    RR_end_point[0] = H;
    RR_end_point[1] = cos(RR_theta)*SL/2 - L1;
    RR_end_point[2] = sin(RR_theta)*SL/2 + z_offset;    
    //cout<<RR_end_point[0]<<"   "<<RR_end_point[1]<<"   "<<RR_end_point[2]<<"     "<<swing<<"\n";



    // For Front Right
    vector<double> FR_end_point = {0,0,0};
    double FR_theta{};
    if(swing){
        if(req_vel[1]>0) FR_theta = mtn_angles[0][1];
        else FR_theta = mtn_angles[1][1];
    }
    else{
        if(req_vel[1]>0) FR_theta = mtn_angles[1][1];
        else FR_theta = mtn_angles[0][1];   
    }
    y = angular_velocity*cos(FR_theta);
    if(swing) z = angular_velocity*sin(FR_theta) - req_vel[0];
    else z = angular_velocity*sin(FR_theta) + req_vel[0];
    FR_theta = atan2(z,y);
    FR_end_point[0] = H;
    FR_end_point[1] = cos(FR_theta)*SL/2 - L1;
    FR_end_point[2] = sin(FR_theta)*SL/2+ z_offset;
    //cout<<FR_end_point[0]<<"   "<<FR_end_point[1]<<"   "<<FR_end_point[2]<<"     "<<1-swing<<"\n";

    // For Rear Left
    vector<double> RL_end_point = {0,0,0};
    double RL_theta{};
    if(swing){
        if(req_vel[1]>0) RL_theta = mtn_angles[0][2];
        else RL_theta = mtn_angles[1][2];
    }
    else{
        if(req_vel[1]>0) RL_theta = mtn_angles[1][2];
        else RL_theta = mtn_angles[0][2];   
    }
    y = angular_velocity*cos(RL_theta);
    if(swing) z = angular_velocity*sin(RL_theta) - req_vel[0];
    else  z = angular_velocity*sin(RL_theta) + req_vel[0];
    RL_theta = atan2(z,y);
    RL_end_point[0] = H;
    RL_end_point[1] = cos(RL_theta)*SL/2 + L1;
    RL_end_point[2] = sin(RL_theta)*SL/2+ z_offset;
    //cout<<RL_end_point[0]<<"   "<<RL_end_point[1]<<"   "<<RL_end_point[2]<<"     "<<1-swing<<"\n";
    

    velocity = sqrt(pow(y,2) + pow(z,2));
    T = SL/velocity;


    vector<vector<double>> out = {FL_end_point,FR_end_point,RL_end_point,RR_end_point};
    return out;
}



