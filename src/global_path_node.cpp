#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include "m545_path_utils/Helpers.hpp"

#include "m545_coverage_planner_ros/GlobalPathRos.hpp"
using namespace m545_coverage_planner_ros;

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_path_node");
  ros::NodeHandle nh;

  GlobalPathRos GlobalPathRos(nh);
  std::vector<geometry_msgs::Pose> globalPath;

  for (int i = 0; i < 3; i++) {
    geometry_msgs::Pose pose;
    pose.position.x = i;
    pose.position.y = 0;
    pose.position.z = 0;
    Eigen::Quaternion<double> quat = m545_path_utils::eulerToQuaternion(0, 0, 0);
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    globalPath.push_back(pose);
  }

  GlobalPathRos.loadPath(globalPath);
  bool startFromCurrentPose = true;
  GlobalPathRos.requestPlanCurrentSegment(startFromCurrentPose);
  // sleep for 10 second;
  ros::Duration(10).sleep();
  GlobalPathRos.requestStartTracking();
  ros::spin();
  return 0;
}