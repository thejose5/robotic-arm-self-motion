#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_listener.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <iostream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
Eigen::Matrix<double,6,7> calcJacobian(tf::TransformListener &listener);
sensor_msgs::JointState joint_state;

void generatePoints(trajectory_msgs::JointTrajectory &traj_command, int num_pts, double joint_vels[7]){
  //Assuming all joints are at 0
  trajectory_msgs::JointTrajectoryPoint point0;
  traj_command.points.push_back(point0);

  traj_command.points[0].velocities.push_back(0);
  traj_command.points[0].velocities.push_back(0);
  traj_command.points[0].velocities.push_back(0);
  traj_command.points[0].velocities.push_back(0);
  traj_command.points[0].velocities.push_back(0);
  traj_command.points[0].velocities.push_back(0);
  traj_command.points[0].velocities.push_back(0);

  traj_command.points[0].positions.push_back(0);
  traj_command.points[0].positions.push_back(0);
  traj_command.points[0].positions.push_back(0);
  traj_command.points[0].positions.push_back(0);
  traj_command.points[0].positions.push_back(0);
  traj_command.points[0].positions.push_back(0);
  traj_command.points[0].positions.push_back(0);

  ros::Duration dur(1);
  traj_command.points[num_pts-1].time_from_start = dur;

  for(int i=1; i<num_pts-1; i++){
    trajectory_msgs::JointTrajectoryPoint point;
    traj_command.points.push_back(point);
    traj_command.points[i].velocities.push_back(joint_vels[0]);
    traj_command.points[i].velocities.push_back(joint_vels[1]);
    traj_command.points[i].velocities.push_back(joint_vels[2]);
    traj_command.points[i].velocities.push_back(joint_vels[3]);
    traj_command.points[i].velocities.push_back(joint_vels[4]);
    traj_command.points[i].velocities.push_back(joint_vels[5]);
    traj_command.points[i].velocities.push_back(joint_vels[6]);

    traj_command.points[i].positions.push_back(joint_vels[0]*(i+1));
    traj_command.points[i].positions.push_back(joint_vels[1]*(i+1));
    traj_command.points[i].positions.push_back(joint_vels[2]*(i+1));
    traj_command.points[i].positions.push_back(joint_vels[3]*(i+1));
    traj_command.points[i].positions.push_back(joint_vels[4]*(i+1));
    traj_command.points[i].positions.push_back(joint_vels[5]*(i+1));
    traj_command.points[i].positions.push_back(joint_vels[6]*(i+1));

    ros::Duration d(i+1);
    traj_command.points[i].time_from_start = d;
  }
  trajectory_msgs::JointTrajectoryPoint point;
  traj_command.points.push_back(point);

  traj_command.points[num_pts-1].velocities.push_back(0);
  traj_command.points[num_pts-1].velocities.push_back(0);
  traj_command.points[num_pts-1].velocities.push_back(0);
  traj_command.points[num_pts-1].velocities.push_back(0);
  traj_command.points[num_pts-1].velocities.push_back(0);
  traj_command.points[num_pts-1].velocities.push_back(0);
  traj_command.points[num_pts-1].velocities.push_back(0);

  traj_command.points[num_pts-1].positions.push_back(joint_vels[0]*(num_pts));
  traj_command.points[num_pts-1].positions.push_back(joint_vels[1]*(num_pts));
  traj_command.points[num_pts-1].positions.push_back(joint_vels[2]*(num_pts));
  traj_command.points[num_pts-1].positions.push_back(joint_vels[3]*(num_pts));
  traj_command.points[num_pts-1].positions.push_back(joint_vels[4]*(num_pts));
  traj_command.points[num_pts-1].positions.push_back(joint_vels[5]*(num_pts));
  traj_command.points[num_pts-1].positions.push_back(joint_vels[6]*(num_pts));

  ros::Duration d(num_pts);
  traj_command.points[num_pts-1].time_from_start = d;
}

void initializeJState(const sensor_msgs::JointState::ConstPtr &msg){
  joint_state = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "self_motion_commander");
  ros::NodeHandle n;
  ros::Publisher traj_command_pub = n.advertise<trajectory_msgs::JointTrajectory>("/kuka_lwr/arm_controller/command", 1000);
  ros::Rate loop_rate(10);

  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle tf_node;
  tf::TransformListener listener;
  ros::Rate rate(10.0);

  ros::init(argc, argv, "self_motion_commander");
  ros::NodeHandle subs_node;
  ros::Subscriber joint_state_sub = subs_node.subscribe("/kuka_lwr/joint_states", 1000, initializeJState);


  ros::Duration(1.0).sleep();

  trajectory_msgs::JointTrajectory traj_command;
  traj_command.joint_names.push_back("kuka_arm_0_joint");
  traj_command.joint_names.push_back("kuka_arm_1_joint");
  traj_command.joint_names.push_back("kuka_arm_2_joint");
  traj_command.joint_names.push_back("kuka_arm_3_joint");
  traj_command.joint_names.push_back("kuka_arm_4_joint");
  traj_command.joint_names.push_back("kuka_arm_5_joint");
  traj_command.joint_names.push_back("kuka_arm_6_joint");

  // Finding joint velocity
  std::vector<double> joint_velocity = joint_state.velocity;
  double* joint_velocity_ptr = joint_velocity.data();
  for(int i=0; i<7; i++){
    std::cout<<joint_velocity[i];
  }
  // Eigen::VectorXd joint_velocity_vector = Eigen::Map<Eigen::VectorXd>(joint_velocity_ptr,7);
  // // Finding jacobian
  // Eigen::MatrixXd jacobian = calcJacobian(listener);
  // //Finding pseudoinverse of jacobian
  // Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
  // //Identity matrix
  // Eigen::MatrixXd I_7 = Eigen::MatrixXd::Identity(7,7);
  // //Z
  // Eigen::Matrix<double,7,1> Z;
  // Z(0,0)=0.5; Z(1,0)=0.5; Z(2,0)=0.5; Z(3,0)=0.5; Z(4,0)=0.5; Z(5,0)=0.5; Z(6,0)=0.5;
  //
  // Eigen::MatrixXd qdot = jacobian_pinv*(jacobian*joint_velocity_vector) + (jacobian_pinv*jacobian - I_7)*Z;
  // double *q_dot;
  // Eigen::Map<Eigen::MatrixXd>( q_dot, qdot.rows(), qdot.cols()) =   qdot;
  // for(int i=0; i<6; i++){
  //   std::cout<<q_dot[i]<<" "<<std::endl;
  // }

  // double joint_vels[7] = {0.2,0.1,0.2,0.2,0.2,0.2,0.2};
  // generatePoints(traj_command,8,q_dot);
  // ros::Rate poll_rate(100);
  //
  // while(traj_command_pub.getNumSubscribers() == 0)
  //   poll_rate.sleep();
  //
  // traj_command_pub.publish(traj_command);
  // ros::spinOnce();
  // loop_rate.sleep();

  return 0;
}
