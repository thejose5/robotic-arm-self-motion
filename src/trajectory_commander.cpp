#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <tf/transform_listener.h>
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
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




int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_commander");
  ros::NodeHandle n;
  ros::Publisher traj_command_pub = n.advertise<trajectory_msgs::JointTrajectory>("/kuka_lwr/arm_controller/command", 1000);
  ros::Rate loop_rate(10);

  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle tf_node;
  tf::TransformListener listener;
  ros::Rate rate(10.0);

  trajectory_msgs::JointTrajectory traj_command;
  traj_command.joint_names.push_back("kuka_arm_0_joint");
  traj_command.joint_names.push_back("kuka_arm_1_joint");
  traj_command.joint_names.push_back("kuka_arm_2_joint");
  traj_command.joint_names.push_back("kuka_arm_3_joint");
  traj_command.joint_names.push_back("kuka_arm_4_joint");
  traj_command.joint_names.push_back("kuka_arm_5_joint");
  traj_command.joint_names.push_back("kuka_arm_6_joint");


  double joint_vels[7] = {0.2,0.1,0.2,0.2,0.2,0.2,0.2};     //[traj_command.joint_names.size()];
  generatePoints(traj_command,8,joint_vels);

  // std::cout<<traj_command.points[3].positions[0]<<" "<<traj_command.points[3].positions[1]<<
  // " "<<traj_command.points[3].positions[2]<<" "<<traj_command.points[3].positions[3];
  ros::Rate poll_rate(100);
while(traj_command_pub.getNumSubscribers() == 0)
    poll_rate.sleep();

  traj_command_pub.publish(traj_command);
  ros::spinOnce();
  loop_rate.sleep();

// Printing the position of end effector regularly
while (tf_node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("calib_kuka_arm_base_link","kuka_arm_7_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s \nTrying Again.",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    std::cout<<"X:"<<transform.getOrigin().x()<<std::endl;
    std::cout<<"Y:"<<transform.getOrigin().y()<<std::endl;
    std::cout<<"Z:"<<transform.getOrigin().z()<<std::endl;
    rate.sleep();
  }

  return 0;
}
