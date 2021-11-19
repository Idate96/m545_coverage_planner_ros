#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include "m545_coverage_planner_ros/GlobalPathPlannerRos.hpp"
#include "m545_path_utils/Helpers.hpp"
using namespace m545_coverage_planner_ros;

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_path_node");
  ros::NodeHandle nh;

  GlobalPathPlannerRos global_planner_ros;
  global_planner_ros.initialize(0.);
  
  ros::spin();
  return 0;
}