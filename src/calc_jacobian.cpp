#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <tf/transform_listener.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>

using namespace std;

Eigen::Matrix3d makeDHCompliant(Eigen::Matrix3d r, vector<int> dh){
  Eigen::Matrix3d r_final;
  for(int i=0; i<3; i++){
      r_final(0,i) = dh[i]/abs(dh[i]) * r(0,abs(dh[i])-1);
      r_final(1,i) = dh[i]/abs(dh[i]) * r(1,abs(dh[i])-1);
      r_final(2,i) = dh[i]/abs(dh[i]) * r(2,abs(dh[i])-1);
  }
  return r_final;
}
/*dh is a matrix (n_joints x 3) which contains the transformations required to convert the coordinate
frames to a DH compliant coordinate frames. 1=x, 2=y, 3=z.
Joint 0: Default, Joint 1: (1,3,-2), Joint 2: Default, Joint 3: (1,-3,2), Joint 4: Default, Joint 5: (1,3,-2), Joint 6: Default*/
Eigen::Matrix<double,6,7> calcJacobian(tf::TransformListener &listener){
  int njoints = 7;
  vector<vector<int>> dh{{1,2,3},{1,3,-2},{1,2,3},{1,-3,2},{1,2,3},{1,3,-2},{1,2,3}};
  std::vector<tf::StampedTransform> transforms;
  // std::vector<Eigen::Quaterniond> quaternions;
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Matrix3d> rotation_matrices;
  std::vector<Eigen::Vector3d> zs;
  Eigen::Matrix<double,6,7> jacobian;

  for(int i=0; i<njoints; i++){
    //Finding transform
    tf::StampedTransform transform;
    string target_link = "/kuka_arm_"+to_string(i+1)+"_link";
    try{
      // cout<<target_link<<endl;
      listener.lookupTransform("/calib_kuka_arm_base_link",target_link,
                             ros::Time(0), transform);
       }
    catch (tf::TransformException &ex){
       ROS_ERROR("%s \nTrying Again.",ex.what());
       ros::Duration(1.0).sleep();
       --i;
       continue;
     }
    //Finding quaternion
    float w = transform.getRotation().getW();
    float x = transform.getRotation().getAxis().x() * sqrt(1-w*w); //getAxis returns (x,y,z)/sin(theta/2)
    float y = transform.getRotation().getAxis().y() * sqrt(1-w*w);
    float z = transform.getRotation().getAxis().z() * sqrt(1-w*w);
    // cout<<transform.getRotation().getAngle()<<endl;
    // cout<<transform.getRotation().getAxis().x()<<" "
    //     <<transform.getRotation().getAxis().y()<<" "
    //     <<transform.getRotation().getAxis().z()<<"\n";
    Eigen::Quaterniond quaternion(w,x,y,z);
    //Finding O
    Eigen::Vector3d position(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    //Converting Quarternion to R Matrix
    Eigen::Matrix3d rot_mat(quaternion);
    //Making rotation matrix DH compliant
    rot_mat = makeDHCompliant(rot_mat,dh[i]);
    //Finding z
    Eigen::Vector3d basis(0.0,0.0,1.0);
    Eigen::Vector3d z_axis = rot_mat * basis;

    //Populating all vectors
    transforms.push_back(transform);
    // quaternions.push_back(quaternion);
    positions.push_back(position);
    rotation_matrices.push_back(rot_mat);
    zs.push_back(z_axis);
  }
  for(int i=0; i<njoints; i++){
    Eigen::Vector3d z_cross_o = zs[i].cross(positions[njoints-1] - positions[i]);
    jacobian(0,i) = z_cross_o.x();
    jacobian(1,i) = z_cross_o.y();
    jacobian(2,i) = z_cross_o.z();
    jacobian(3,i) = zs[i].x();
    jacobian(4,i) = zs[i].y();
    jacobian(5,i) = zs[i].z();
  }
  /*************COMMENT OUT AFTER DEBUGGING****************/
  // for(int i=0; i<njoints; i++){
  //   cout<<"------------------"<<endl;
  //   cout<<"Joint "<<i<<endl;
  //   cout<<"------------------"<<endl;
  //   cout<<"Rotation Matrix:"<<endl;
  //   cout<<rotation_matrices[i]<<endl;;
  //   cout<<"Position(O):"<<endl;
  //   cout<<positions[i]<<endl;
  // }
  cout<<"------------------"<<endl;
  cout<<"Jacobian: "<<endl;
  cout<<"------------------\n"<<endl;
  cout<<jacobian<<endl;
  /*******************************************/

  return jacobian;
}

// int main(){
//
//   return 0;
// }
