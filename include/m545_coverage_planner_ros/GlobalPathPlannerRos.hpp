#pragma once
#include <geometry_msgs/Pose.h>
#include <m545_planner_msgs/PathFollowerTrackingStatusRos.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

namespace m545_coverage_planner_ros {

template <typename Req, typename Res>
bool CallService(Req& req, Res& res, const std::string& service_name) {
  try {
    // ROS_DEBUG_STREAM("Service name: " << service_name);
    if (!ros::service::call(service_name, req, res)) {
      ROS_WARN_STREAM("Couldn't call service: " << service_name);
      return false;
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Service Exception: " << e.what());
    return false;
  }
  return true;
}

class GlobalPathPlannerRos {
 public:
  GlobalPathPlannerRos() = default;
  ~GlobalPathPlannerRos();

  void initialize(double dt);
  void initRos();

  void setPath(std::vector<geometry_msgs::Pose>& path);
  const std::vector<geometry_msgs::Pose>& GetPath();
  
  bool requestPlan(geometry_msgs::Pose& start, geometry_msgs::Pose& goal);
  bool requestPlanCurrentSegment(bool start_from_current_pose);
  void requestStartTracking();
  void requestStopTracking();
  void requestPose(geometry_msgs::Pose& pose);
  
  bool completedPath();
  void globalPathCallback(const geometry_msgs::PoseArray& msg);
  void publishPathPoints() const;
  void publishPathPoses() const;

  // ros
  ros::NodeHandle nh_;
  ros::Publisher globalPathPub_;
  ros::Subscriber globalPathSub_;
  ros::Subscriber trackingStatusSub_;

  // params
  std::string control_command_topic_;
  std::string planning_service_name_;
  std::string current_state_service_name_;
  std::string path_topic_;

  // global path
  std::vector<geometry_msgs::Pose> globalPath_;
  unsigned int current_segment_index_ = 0;

};

}  // namespace m545_coverage_planner_ros