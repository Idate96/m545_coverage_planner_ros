#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include "m545_coverage_planner_ros/GlobalPathPlannerRos.hpp"
#include "m545_path_utils/Helpers.hpp"
using namespace m545_coverage_planner_ros;

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_path_node");
  ros::NodeHandle nh;

  GlobalPathPlannerRos global_planner_ros;
  std::vector<geometry_msgs::Pose> global_path;

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
    global_path.push_back(pose);
  }

  global_planner_ros.LoadPath(global_path);
  bool start_from_current_pose = true;
  global_planner_ros.RequestPlanCurrentSegment(start_from_current_pose);
  // sleep for 10 second;
  ros::Duration(10).sleep();
  global_planner_ros.RequestStartTracking();
  ros::spin();
  return 0;
}