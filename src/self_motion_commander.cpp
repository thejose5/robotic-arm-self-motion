#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_listener.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

//Global variables
Eigen::Matrix<double,6,7> calcJacobian(tf::TransformListener &listener);
sensor_msgs::JointState joint_state, empty_msg;
double pi = 3.14159265359;




void generatePoints(trajectory_msgs::JointTrajectory &traj_command, int num_pts,
                          double joint_vels[7], std::vector<double> init_jpos){
  traj_command.joint_names.push_back("kuka_arm_0_joint");
  traj_command.joint_names.push_back("kuka_arm_1_joint");
  traj_command.joint_names.push_back("kuka_arm_2_joint");
  traj_command.joint_names.push_back("kuka_arm_3_joint");
  traj_command.joint_names.push_back("kuka_arm_4_joint");
  traj_command.joint_names.push_back("kuka_arm_5_joint");
  traj_command.joint_names.push_back("kuka_arm_6_joint");
  double time_multiplier = 0.5;
  for(int i=0; i<num_pts-1; i++){
    double time = (i+1)*time_multiplier;
    trajectory_msgs::JointTrajectoryPoint point;
    traj_command.points.push_back(point);
    traj_command.points[i].velocities.push_back(joint_vels[0]);
    traj_command.points[i].velocities.push_back(joint_vels[1]);
    traj_command.points[i].velocities.push_back(joint_vels[2]);
    traj_command.points[i].velocities.push_back(joint_vels[3]);
    traj_command.points[i].velocities.push_back(joint_vels[4]);
    traj_command.points[i].velocities.push_back(joint_vels[5]);
    traj_command.points[i].velocities.push_back(joint_vels[6]);

    traj_command.points[i].positions.push_back(init_jpos[0]+joint_vels[0]*time);
    traj_command.points[i].positions.push_back(init_jpos[1]+joint_vels[1]*time);
    traj_command.points[i].positions.push_back(init_jpos[2]+joint_vels[2]*time);
    traj_command.points[i].positions.push_back(init_jpos[3]+joint_vels[3]*time);
    traj_command.points[i].positions.push_back(init_jpos[4]+joint_vels[4]*time);
    traj_command.points[i].positions.push_back(init_jpos[5]+joint_vels[5]*time);
    traj_command.points[i].positions.push_back(init_jpos[6]+joint_vels[6]*time);

    ros::Duration d(time);
    traj_command.points[i].time_from_start = d;
  }
  //
  // double time = (num_pts)*time_multiplier;
  // trajectory_msgs::JointTrajectoryPoint point;
  // traj_command.points.push_back(point);
  //
  // traj_command.points[num_pts-1].velocities.push_back(0);
  // traj_command.points[num_pts-1].velocities.push_back(0);
  // traj_command.points[num_pts-1].velocities.push_back(0);
  // traj_command.points[num_pts-1].velocities.push_back(0);
  // traj_command.points[num_pts-1].velocities.push_back(0);
  // traj_command.points[num_pts-1].velocities.push_back(0);
  // traj_command.points[num_pts-1].velocities.push_back(0);
  //
  // traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[0]+joint_vels[0]*time_multiplier/2);
  // traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[1]+joint_vels[1]*time_multiplier/2);
  // traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[2]+joint_vels[2]*time_multiplier/2);
  // traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[3]+joint_vels[3]*time_multiplier/2);
  // traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[4]+joint_vels[4]*time_multiplier/2);
  // traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[5]+joint_vels[5]*time_multiplier/2);
  // traj_command.points[num_pts-1].positions.push_back(traj_command.points[num_pts-2].positions[6]+joint_vels[6]*time_multiplier/2);
  //
  // ros::Duration d(time);
  // traj_command.points[num_pts-1].time_from_start = d;
}





void initializeJState(const sensor_msgs::JointState::ConstPtr &msg){
  joint_state = *msg;
}



// void clockCallback(const sensor_msgs::JointState::ConstPtr)



// std::vector<double> findInitJPos(tf::TransformListener &listener){
//   std::cout<<std::endl<<"Init Joint Pos:"<<std::endl;
//   std::vector<double> init_jpos;
//   int njoints = 7;
//   tf::StampedTransform transform;
//   double angle;
//
//   //*********************For Joint 0:
//   while(true){
//     try{
//       listener.lookupTransform("/calib_kuka_arm_base_link","/kuka_arm_1_link",
//                              ros::Time(0), transform);
//       break;
//        }
//     catch (tf::TransformException &ex){
//        ROS_ERROR("%s \nTrying Again.",ex.what());
//        ros::Duration(1.0).sleep();
//        continue;
//      }
//   }
//   angle = transform.getRotation().getAngle();
//   Eigen::Vector3d ones(1,1,1);
//   Eigen::Vector3d axis(transform.getRotation().getAxis().x(),
//                        transform.getRotation().getAxis().y(),
//                        transform.getRotation().getAxis().z());
//   std::cout<<"Axis: \n"<<axis<<std::endl<<std::endl;
//   angle = angle*(abs(axis.dot(ones))/axis.dot(ones));
//   std::cout<<angle<<std::endl;
//   init_jpos.push_back(angle);
//   //**************************************
//
//   for(int i=2; i<=njoints; i++){
//     std::string basis_link = "/kuka_arm_"+std::to_string(i-1)+"_link";
//     std::string target_link = "/kuka_arm_"+std::to_string(i)+"_link";
//     try{
//       listener.lookupTransform(basis_link,target_link, ros::Time(0), transform);
//     }
//     catch (tf::TransformException &ex){
//        ROS_ERROR("%s \nTrying Again.",ex.what());
//        ros::Duration(1.0).sleep();
//        --i;
//        continue;
//     }
//     angle = transform.getRotation().getAngle();
//     Eigen::Vector3d ones(1,1,1);
//     Eigen::Vector3d axis(transform.getRotation().getAxis().x(),
//                          transform.getRotation().getAxis().y(),
//                          transform.getRotation().getAxis().z());
//     std::cout<<"Axis: \n"<<axis<<std::endl<<std::endl;
//     angle = angle*(abs(axis.dot(ones))/axis.dot(ones));
//     std::cout<<angle<<std::endl;
//     init_jpos.push_back(angle);
//   }
//   return init_jpos;
// }




double[] calcQdot(tf::TransformListener &listener)
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
  //Converting qdot to array q_dot
  double q_dot[qdot.size()];
  Eigen::Map<Eigen::MatrixXd>( q_dot, qdot.rows(), qdot.cols()) =   qdot;

  return q_dot;
}





int main(int argc, char **argv)
{
  //******Declaring Nodes*********//
  ros::init(argc, argv, "self_motion_commander");
  ros::NodeHandle n;
  ros::Publisher traj_command_pub = n.advertise<trajectory_msgs::JointTrajectory>("/kuka_lwr/arm_controller/command", 1000);
  ros::Rate loop_rate(0.2);

  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle tf_node;
  tf::TransformListener listener;
  ros::Rate rate(10.0);

  ros::init(argc, argv, "self_motion_commander");
  ros::NodeHandle subs_node;
  ros::Subscriber joint_state_sub = subs_node.subscribe("kuka_lwr/joint_states", 0, initializeJState);

  // ros::init(argc, argv, "clock_listener");
  // ros::NodeHandle clock_listener_node;
  // ros::Subscriber clock = clock_listener_node.subscribe("/clock", 0, clockCallback);
  //******************************//


  while(joint_state == empty_msg)
    ros::spinOnce();

  trajectory_msgs::JointTrajectory traj_command;

  std::vector<double> init_jpos = joint_state.position;
  std::cout<<init_jpos[0]<<" "<<init_jpos[1]<<" "<<init_jpos[2]<<" "<<init_jpos[3]<<" "
  <<init_jpos[4]<<" "<<init_jpos[5]<<" "<<init_jpos[6]<<"\n";

  generatePoints(traj_command,8,calcQdot,init_jpos);
  ros::Rate poll_rate(100);

  while(traj_command_pub.getNumSubscribers() == 0)
    poll_rate.sleep();

  traj_command_pub.publish(traj_command);
  ros::spinOnce();


  return 0;
}
