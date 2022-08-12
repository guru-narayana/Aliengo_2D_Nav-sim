//#include "unitree_legged_msgs/Aliengo_Joint_controll.h"

#include "Kinematics.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_generator.h"
#include "unitree_legged_msgs/Aliengo_Joint_controll.h"


// variables that update regurarly via callback
vector<double> req_vel = {0,0};      // required velocity of the robot { 0 m/s : front and back, 0 rad/sec rotational}
vector<double> FL_current_xyz_st = {0,0,0}; // actual  Front Left foot position wrt thighs coordinate frame
vector<double> RL_current_xyz_st = {0,0,0};
vector<double> FR_current_xyz_st = {0,0,0};
vector<double> RR_current_xyz_st = {0,0,0}; // actual Rear Right foot position wrt thigh coordinate frame

// target variables set by user
double SL = 0.2; // step length , This is the max step length set by user
double SH = 0.03; // step height
double Height = 0.33; // height at which robot need to stand


// varible for storing the publisher object
ros::Publisher jnt_st_pub;

// variable that stores the jnt states to be published
unitree_legged_msgs::Aliengo_Joint_controll jnt_set_st;

// callback Functions
void req_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    req_vel[0] = msg->linear.x;
    req_vel[1] = msg->angular.z; 
    SL = min(0.2,sqrt(pow(req_vel[0],2) + pow(req_vel[1],2))*0.1); // Changes step length depending on velocity to keep robot stable
}
void joint_current_act_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    vector<double> FL_current_jnt_st = {0,0,0}; //actual joint state of the Front Left side leg
    vector<double> RL_current_jnt_st = {0,0,0}; 
    vector<double> FR_current_jnt_st = {0,0,0};
    vector<double> RR_current_jnt_st = {0,0,0}; //actual joint state of the Rear Right side leg
    // joint matrix order : [FL_calf_joint, FL_hip_joint, FL_thigh_joint, FR_calf_joint, FR_hip_joint, FR_thigh_joint,RL_calf_joint, 
    //                                                RL_hip_joint, RL_thigh_joint, RR_calf_joint, RR_hip_joint, RR_thigh_joint]
    FL_current_jnt_st[0] = (double)msg->position[1];
    FL_current_jnt_st[1] = (double)msg->position[2];
    FL_current_jnt_st[2] = (double)msg->position[0];
    
    FR_current_jnt_st[0] = (double)msg->position[1+3];
    FR_current_jnt_st[1] = -(double)msg->position[2+3];
    FR_current_jnt_st[2] = -(double)msg->position[0+3];

    RL_current_jnt_st[0] = (double)msg->position[1+6];
    RL_current_jnt_st[1] = (double)msg->position[2+6];
    RL_current_jnt_st[2] = (double)msg->position[0+6];

    RR_current_jnt_st[0] = (double)msg->position[1+9];
    RR_current_jnt_st[1] = -(double)msg->position[2+9];
    RR_current_jnt_st[2] = -(double)msg->position[0+9];


    FL_current_xyz_st = Left_Leg_FK(FL_current_jnt_st); // converting the jnt state to the xyz state
    RL_current_xyz_st = Left_Leg_FK(RL_current_jnt_st);
    FR_current_xyz_st = Right_Leg_FK(FR_current_jnt_st);
    RR_current_xyz_st = Right_Leg_FK(RR_current_jnt_st);
    //cout<<FR_current_xyz_st[0]<<"  " <<FR_current_xyz_st[1]<<"   "<<FR_current_xyz_st[2]<<"\n";
}


// One time requirement function matrix multiplication
vector<vector<double>> Multiply(vector <vector<double>> &a, vector <vector<double>> &b)
{
    const int n = a.size();     // a rows
    const int m = a[0].size();  // a cols
    const int p = b[0].size();  // b cols
    vector <vector<double>> c(n, vector<double>(p, 0));
    for (auto j = 0; j < p; ++j)
    {
        for (auto k = 0; k < m; ++k)
        {
            for (auto i = 0; i < n; ++i)
            {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return c;
}


void Trajectory_exec(vector<vector<double>> Trajectory_end_points,int swing){
    ros::Rate loop_rate(500);
    if(swing){
        vector<double> FL_1 = FL_current_xyz_st;
        vector<double> FL_3 = Trajectory_end_points[0];
        vector<double> FL_2 = {0,0,0};
        FL_2[0] = ((FL_1[0] + FL_3[0])/2) - SH; 
        FL_2[2] = (FL_1[2] + FL_3[2])/2;
        FL_2[1] = (FL_1[1] + FL_3[1])/2;
        vector<vector<double>> FL_x = {{FL_1[0],FL_2[0],FL_3[0]},
                                       {FL_1[1],FL_2[1],FL_3[1]},   
                                       {FL_1[2],FL_2[2],FL_3[2]}};
        vector<vector<double>> FL_XB = Multiply(FL_x,blending_matrix);

        vector<double> RR_1 = RR_current_xyz_st;
        vector<double> RR_3 = Trajectory_end_points[3];
        vector<double> RR_2 = {0,0,0};
        RR_2[0] = ((RR_1[0] + RR_3[0])/2)-SH; 
        RR_2[2] = (RR_1[2] + RR_3[2])/2;
        RR_2[1] = (RR_1[1] + RR_3[1])/2;
        vector<vector<double>> RR_x = {{RR_1[0],RR_2[0],RR_3[0]},
                                       {RR_1[1],RR_2[1],RR_3[1]},   
                                       {RR_1[2],RR_2[2],RR_3[2]}};
        vector<vector<double>> RR_XB = Multiply(RR_x,blending_matrix);

        vector<double> FR_1 = FR_current_xyz_st;
        vector<double> FR_2 = Trajectory_end_points[1];

        vector<double> RL_1 = RL_current_xyz_st;
        vector<double> RL_2 = Trajectory_end_points[2];

        double start_time = ros::Time::now().toSec();
        //cout<<T<<"\n";
        while( (ros::Time::now().toSec()-start_time < T) && ros::ok()){
            // variable that change per instant
            ros::spinOnce();
            double u = (ros::Time::now().toSec()-start_time)/T;
            vector<double> FL_req_pos = {0,0,0};
            vector<double> FR_req_pos = {0,0,0};
            vector<double> RL_req_pos = {0,0,0};
            vector<double> RR_req_pos = {0,0,0};
            vector<vector<double>> U_mat = {{1},{u},{pow(u,2)}};
            


            // Front left current joint position estimation
            vector<vector<double>> FL_XBU = Multiply(FL_XB,U_mat);
            FL_req_pos[0] = FL_XBU[0][0];
            FL_req_pos[1] = FL_XBU[1][0];
            FL_req_pos[2] = FL_XBU[2][0];

            vector<double>FL_req_jnt = Left_Leg_IK(FL_req_pos);
            jnt_set_st.joint_positions[3] = FL_req_jnt[0];
            jnt_set_st.joint_positions[4] = FL_req_jnt[1];
            jnt_set_st.joint_positions[5] = FL_req_jnt[2];


            // Rear Right current joint position estimation
            vector<vector<double>> RR_XBU = Multiply(RR_XB,U_mat);
            RR_req_pos[0] = RR_XBU[0][0];
            RR_req_pos[1] = RR_XBU[1][0];
            RR_req_pos[2] = RR_XBU[2][0];
            //cout<<FL_req_pos[0]<<"   "  <<FL_req_pos[1]<<"   "<<FL_req_pos[2]<<"\n";
            vector<double>RR_req_jnt = Right_Leg_IK(RR_req_pos);
            jnt_set_st.joint_positions[6] = -RR_req_jnt[0];
            jnt_set_st.joint_positions[7] = -RR_req_jnt[1];
            jnt_set_st.joint_positions[8] = -RR_req_jnt[2];


            // Front Right current joint position estimation
            FR_req_pos[0] = FR_1[0] *(1-u) + FR_2[0]*u;
            FR_req_pos[1] = FR_1[1] *(1-u) + FR_2[1]*u;
            FR_req_pos[2] = FR_1[2] *(1-u) + FR_2[2]*u;
            vector<double>FR_req_jnt = Right_Leg_IK(FR_req_pos);
            jnt_set_st.joint_positions[0] = -FR_req_jnt[0];
            jnt_set_st.joint_positions[1] = -FR_req_jnt[1];
            jnt_set_st.joint_positions[2] = -FR_req_jnt[2];


            // Rear Left current joint position estimation
            RL_req_pos[0] = RL_1[0] *(1-u) + RL_2[0]*u;
            RL_req_pos[1] = RL_1[1] *(1-u) + RL_2[1]*u;
            RL_req_pos[2] = RL_1[2] *(1-u) + RL_2[2]*u;
            vector<double>RL_req_jnt = Left_Leg_IK(RL_req_pos);
            jnt_set_st.joint_positions[9] = RL_req_jnt[0];
            jnt_set_st.joint_positions[10] = RL_req_jnt[1];
            jnt_set_st.joint_positions[11] = RL_req_jnt[2];

            jnt_st_pub.publish(jnt_set_st);
            loop_rate.sleep();

        }

        

    }
    else{
        vector<double> FR_1 = FR_current_xyz_st;
        vector<double> FR_3 = Trajectory_end_points[1];
        vector<double> FR_2 = {0,0,0};
        FR_2[0] = (FR_1[0] + FR_3[0])/2 - SH; 
        FR_2[2] = (FR_1[2] + FR_3[2])/2;
        FR_2[1] = (FR_1[1] + FR_3[1])/2;
        vector<vector<double>> FR_x = {{FR_1[0],FR_2[0],FR_3[0]},
                                       {FR_1[1],FR_2[1],FR_3[1]},   
                                       {FR_1[2],FR_2[2],FR_3[2]}};
        vector<vector<double>> FR_XB = Multiply(FR_x,blending_matrix);
        
        vector<double> RL_1 = RL_current_xyz_st;
        vector<double> RL_3 = Trajectory_end_points[2];
        vector<double> RL_2 = {0,0,0};
        RL_2[0] = (RL_1[0] + RL_3[0])/2 - SH; 
        RL_2[2] = (RL_1[2] + RL_3[2])/2;
        RL_2[1] = (RL_1[1] + RL_3[1])/2;
        vector<vector<double>> RL_x = {{RL_1[0],RL_2[0],RL_3[0]},
                                       {RL_1[1],RL_2[1],RL_3[1]},   
                                       {RL_1[2],RL_2[2],RL_3[2]}};
        vector<vector<double>> RL_XB = Multiply(RL_x,blending_matrix);

        vector<double> FL_1 = FL_current_xyz_st;
        vector<double> FL_2 = Trajectory_end_points[0];

        vector<double> RR_1 = RR_current_xyz_st;
        vector<double> RR_2 = Trajectory_end_points[3];

        double start_time = ros::Time::now().toSec();
        //cout<<T<<"\n";
        //cout<<FL_1[0]<<"  "<<FL_2[0]<<"\n";
        while( ((ros::Time::now().toSec()-start_time) < T) && ros::ok()){
            // variable that change per instant
            ros::spinOnce();
            double u = (ros::Time::now().toSec()-start_time)/T;
            vector<double> FL_req_pos = {0,0,0};
            vector<double> FR_req_pos = {0,0,0};
            vector<double> RL_req_pos = {0,0,0};
            vector<double> RR_req_pos = {0,0,0};
            vector<vector<double>> U_mat = {{1},{u},{pow(u,2)}};
            


            // Front left current joint position estimation
            vector<vector<double>> FR_XBU = Multiply(FR_XB,U_mat);
            FR_req_pos[0] = FR_XBU[0][0];
            FR_req_pos[1] = FR_XBU[1][0];
            FR_req_pos[2] = FR_XBU[2][0];
            //cout<<FL_req_pos[0]<<"   "  <<FL_req_pos[1]<<"   "<<FL_req_pos[2]<<"\n";
            vector<double>FR_req_jnt = Right_Leg_IK(FR_req_pos);
            jnt_set_st.joint_positions[0] = -FR_req_jnt[0];
            jnt_set_st.joint_positions[1] = -FR_req_jnt[1];
            jnt_set_st.joint_positions[2] = -FR_req_jnt[2];


            // Rear Right current joint position estimation
            vector<vector<double>> RL_XBU = Multiply(RL_XB,U_mat);
            RL_req_pos[0] = RL_XBU[0][0];
            RL_req_pos[1] = RL_XBU[1][0];
            RL_req_pos[2] = RL_XBU[2][0];
            //cout<<FL_req_pos[0]<<"   "  <<FL_req_pos[1]<<"   "<<FL_req_pos[2]<<"\n";
            vector<double>RL_req_jnt = Left_Leg_IK(RL_req_pos);
            jnt_set_st.joint_positions[9] = RL_req_jnt[0];
            jnt_set_st.joint_positions[10] = RL_req_jnt[1];
            jnt_set_st.joint_positions[11] = RL_req_jnt[2];


            // Front Right current joint position estimation
            FL_req_pos[0] = FL_1[0] *(1-u) + FL_2[0]*u;
            FL_req_pos[1] = FL_1[1] *(1-u) + FL_2[1]*u;
            FL_req_pos[2] = FL_1[2] *(1-u) + FL_2[2]*u;
            vector<double>FL_req_jnt = Left_Leg_IK(FL_req_pos);
            jnt_set_st.joint_positions[3] = FL_req_jnt[0];
            jnt_set_st.joint_positions[4] = FL_req_jnt[1];
            jnt_set_st.joint_positions[5] = FL_req_jnt[2];


            // Rear Left current joint position estimation
            RR_req_pos[0] = RR_1[0] *(1-u) + RR_2[0]*u;
            RR_req_pos[1] = RR_1[1] *(1-u) + RR_2[1]*u;
            RR_req_pos[2] = RR_1[2] *(1-u) + RR_2[2]*u;
            vector<double>RR_req_jnt = Right_Leg_IK(RR_req_pos);
            jnt_set_st.joint_positions[6] = -RR_req_jnt[0];
            jnt_set_st.joint_positions[7] = -RR_req_jnt[1];
            jnt_set_st.joint_positions[8] = -RR_req_jnt[2];

            jnt_st_pub.publish(jnt_set_st);
            loop_rate.sleep();
        }
    }

}

int main(int argc,char** argv){
    ros::init(argc, argv, "Aliengo_gait_Controller");
    ros::NodeHandle n;
    cout<<"Starting the gait Controller ...."<<"\n";
    // Publishing the jnt_state from the trajectory
    jnt_st_pub = n.advertise<unitree_legged_msgs::Aliengo_Joint_controll>("Aliengo_jnt_req_state", 1);
    // subcriber to current velocity required
    ros::Subscriber req_vel_sub = n.subscribe("/cmd_vel", 1, req_vel_callback); 
    // subcriber to current joint state of the robot
    ros::Subscriber crnt_jnt_st_sub = n.subscribe("/aliengo_gazebo/joint_states", 1, joint_current_act_state_callback); 
    cout<<"Topic subscrition and publication Successful"<<"\n";

    // one time run functions
    vector<vector<double>> Robot_angular_mtn_angles = Robot_angular_motion_endpoints(L1,Length,width,SL);
    cout<<"Twist angle estimation Successful"<<"\n";

    int swing = 1; // swing control variable that controls if LF and RR swings or stance
    cout<<"Gait controller initialised use ctrl-c to quit "<<"\n";
    while(ros::ok()){
        ros::spinOnce();
        vector<vector<double>> Trajectory_end_points = Robot_end_points(req_vel,Robot_angular_mtn_angles,swing,SL,Height);
        Trajectory_exec(Trajectory_end_points,swing);
        swing = 1-swing;
        
    }
    return 0;
    
}