#pragma once
#include <geometry_msgs/Pose.h>
#include <m545_planner_msgs/PathFollowerTrackingStatusRos.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

namespace m545_coverage_planner_ros {

template <typename Req, typename Res>
bool callService(Req& req, Res& res, const std::string& serviceName) {
  try {
    // ROS_DEBUG_STREAM("Service name: " << service_name);
    if (!ros::service::call(serviceName, req, res)) {
      ROS_WARN_STREAM("Couldn't call service: " << serviceName);
      return false;
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Service Exception: " << e.what());
    return false;
  }
  return true;
}

class GlobalPathRos {
 public:
  GlobalPathRos(ros::NodeHandle& nh);
  ~GlobalPathRos();

  void trackingStatusCallback(const m545_planner_msgs::PathFollowerTrackingStatusRos& msg);
  void publishGlobalPath();
  void loadPathFromFile(std::string filename);
  void setPath(std::vector<geometry_msgs::Pose>& path);
  void requestPlan(geometry_msgs::Pose& start, geometry_msgs::Pose& goal);
  void requestPlanCurrentSegment(bool startFromCurrentPose);
  void requestStartTracking();
  void requestStopTracking();
  void requestPose(geometry_msgs::Pose& pose);
  bool completedPath();
  void loadPath(std::vector<geometry_msgs::Pose>& path);

  // ros
  ros::NodeHandle nh_;
  ros::Publisher globalPathPub_;
  ros::Subscriber trackingStatusSub_;

  // params
  std::string control_command_topic_;
  std::string planning_service_name_;
  std::string current_state_service_name_;

  // global path
  std::vector<geometry_msgs::Pose> globalPath_;
  int currentSegmentIndex_ = 0;
  bool prevTracking_ = false;
  bool tracking_ = false;
};

}  // namespace m545_coverage_planner_ros