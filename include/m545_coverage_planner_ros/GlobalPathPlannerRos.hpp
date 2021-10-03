#pragma once
#include <geometry_msgs/Pose.h>
#include <m545_planner_msgs/PathFollowerTrackingStatusRos.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include "m545_path_utils/Helpers.hpp"

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

  void Initialize(double dt);
  void InitRos();
  void PublishGlobalPath();
  void LoadPathFromFile(std::string filename);
  void SetPath(std::vector<geometry_msgs::Pose>& path);
  const std::vector<geometry_msgs::Pose>& GetPath();
  bool RequestPlan(geometry_msgs::Pose& start, geometry_msgs::Pose& goal);
  bool RequestPlanCurrentSegment(bool start_from_current_pose);
  void RequestStartTracking();
  void RequestStopTracking();
  void RequestPose(geometry_msgs::Pose& pose);
  bool CompletedPath();
  void LoadPath(std::vector<geometry_msgs::Pose>& path);
  const std::vector<geometry_msgs::Pose>& getPath();
  void LoadDummyPath();

  // ros
  ros::NodeHandle nh_;
  ros::Publisher global_path_pub_;
  ros::Subscriber tracking_status_sub_;

  // params
  std::string control_command_topic_;
  std::string planning_service_name_;
  std::string current_state_service_name_;

  // global path
  std::vector<geometry_msgs::Pose> global_path_;
  unsigned int current_segment_index_ = 0;
};

}  // namespace m545_coverage_planner_ros