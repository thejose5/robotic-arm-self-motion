#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf/transform_listener.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

//Global variables
Eigen::Matrix<double,6,7> calcJacobian(tf::TransformListener &listener);
sensor_msgs::JointState joint_state, empty_msg;
double pi = 3.14159265359;


// void generatePoints(trajectory_msgs::JointTrajectory &traj_command, int num_pts,
//                           double joint_vels[7], std::vector<double> init_jpos){
//   double time_multiplier = 0.5;
//   for(int i=0; i<num_pts-1; i++){
//     double time = (i+1)*time_multiplier;
//     trajectory_msgs::JointTrajectoryPoint point;
//     traj_command.points.push_back(point);
//     traj_command.points[i].velocities.push_back(joint_vels[0]);
//     traj_command.points[i].velocities.push_back(joint_vels[1]);
//     traj_command.points[i].velocities.push_back(joint_vels[2]);
//     traj_command.points[i].velocities.push_back(joint_vels[3]);
//     traj_command.points[i].velocities.push_back(joint_vels[4]);
//     traj_command.points[i].velocities.push_back(joint_vels[5]);
//     traj_command.points[i].velocities.push_back(joint_vels[6]);
//
//     traj_command.points[i].positions.push_back(init_jpos[0]+joint_vels[0]*time);
//     traj_command.points[i].positions.push_back(init_jpos[1]+joint_vels[1]*time);
//     traj_command.points[i].positions.push_back(init_jpos[2]+joint_vels[2]*time);
//     traj_command.points[i].positions.push_back(init_jpos[3]+joint_vels[3]*time);
//     traj_command.points[i].positions.push_back(init_jpos[4]+joint_vels[4]*time);
//     traj_command.points[i].positions.push_back(init_jpos[5]+joint_vels[5]*time);
//     traj_command.points[i].positions.push_back(init_jpos[6]+joint_vels[6]*time);
//
//     ros::Duration d(time);
//     traj_command.points[i].time_from_start = d;
//   }
//
//   double time = (num_pts)*time_multiplier;
//   trajectory_msgs::JointTrajectoryPoint point;
//   traj_command.points.push_back(point);
//
//   traj_command.points[num_pts-1].velocities.push_back(0);
//   traj_command.points[num_pts-1].velocities.push_back(0);
//   traj_command.points[num_pts-1].velocities.push_back(0);
//   traj_command.points[num_pts-1].velocities.push_back(0);
//   traj_command.points[num_pts-1].velocities.push_back(0);
//   traj_command.points[num_pts-1].velocities.push_back(0);
//   traj_command.points[num_pts-1].velocities.push_back(0);
//
//   traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[0]+joint_vels[0]*time_multiplier/2);
//   traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[1]+joint_vels[1]*time_multiplier/2);
//   traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[2]+joint_vels[2]*time_multiplier/2);
//   traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[3]+joint_vels[3]*time_multiplier/2);
//   traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[4]+joint_vels[4]*time_multiplier/2);
//   traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[5]+joint_vels[5]*time_multiplier/2);
//   traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[6]+joint_vels[6]*time_multiplier/2);
//
//   ros::Duration d(time);
//   traj_command.points[num_pts-1].time_from_start = d;
// }





// void callback(const sensor_msgs::JointState::ConstPtr &msg){
//
// }




Eigen::MatrixXd calcQdot(tf::TransformListener &listener)
{
  // Finding jacobian
  Eigen::MatrixXd jacobian = calcJacobian(listener);
  //Finding pseudoinverse of jacobian
  Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
  //Identity matrix
  Eigen::MatrixXd I_7 = Eigen::MatrixXd::Identity(7,7);
  //Z
  Eigen::Matrix<double,7,1> Z;
  Z(0,0)=4; Z(1,0)=4; Z(2,0)=4; Z(3,0)=4; Z(4,0)=4; Z(5,0)=4; Z(6,0)=4;

  Eigen::MatrixXd qdot = (jacobian_pinv*jacobian - I_7)*Z;
  std::cout<<"\nQDot:\n";
  std::cout<<qdot<<std::endl;

  return qdot;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "self_motion_commander_v2");
  ros::NodeHandle n;
  ros::Publisher joint0_vel_command = n.advertise<std_msgs::Float64>("/kuka_lwr/joint0_velocity_controller/command", 1000);
  ros::Publisher joint1_vel_command = n.advertise<std_msgs::Float64>("/kuka_lwr/joint1_velocity_controller/command", 1000);
  ros::Publisher joint2_vel_command = n.advertise<std_msgs::Float64>("/kuka_lwr/joint2_velocity_controller/command", 1000);
  ros::Publisher joint3_vel_command = n.advertise<std_msgs::Float64>("/kuka_lwr/joint3_velocity_controller/command", 1000);
  ros::Publisher joint4_vel_command = n.advertise<std_msgs::Float64>("/kuka_lwr/joint4_velocity_controller/command", 1000);
  ros::Publisher joint5_vel_command = n.advertise<std_msgs::Float64>("/kuka_lwr/joint5_velocity_controller/command", 1000);
  ros::Publisher joint6_vel_command = n.advertise<std_msgs::Float64>("/kuka_lwr/joint6_velocity_controller/command", 1000);

  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle tf_node;
  tf::TransformListener listener;
  ros::Rate loop_rate(10.0);

  int i=0;
  while(i<50){
    //Getting qdot
    Eigen::MatrixXd qdot = calcQdot(listener);
    //Converting qdot to array q_dot
    double q_dot[qdot.size()];
    Eigen::Map<Eigen::MatrixXd>( q_dot, qdot.rows(), qdot.cols()) =   qdot; 
    //Converting array q_dot to std_msgs::Float64
    std_msgs::Float64 qdot_msg[qdot.size()];
    for(int i=0; i<qdot.size(); i++){
      qdot_msg[i].data = (q_dot[i]);
    }

    //Publishing results
    joint0_vel_command.publish(qdot_msg[0]);
    joint1_vel_command.publish(qdot_msg[1]);
    joint2_vel_command.publish(qdot_msg[2]);
    joint3_vel_command.publish(qdot_msg[3]);
    joint4_vel_command.publish(qdot_msg[4]);
    joint5_vel_command.publish(qdot_msg[5]);
    joint6_vel_command.publish(qdot_msg[6]);

    i++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
