#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <tf/transform_listener.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

Eigen::Matrix<float,6,7> calcJacobian(tf::TransformListener &listener);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle tf_node;
  tf::TransformListener lstnr;
  ros::Rate rate(10.0);

  calcJacobian(lstnr);

  return 0;
}
