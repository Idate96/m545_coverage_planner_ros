#include "m545_coverage_planner_ros/GlobalPathPlannerRos.hpp"

#include <se2_navigation_msgs/RequestPathSrv.h>

#include <fstream>

#include "se2_navigation_msgs/ControllerCommand.hpp"
#include "se2_navigation_msgs/RequestCurrentStateSrv.h"
#include "se2_navigation_msgs/SendControllerCommandSrv.h"

namespace m545_coverage_planner_ros {

GlobalPathPlannerRos::~GlobalPathPlannerRos() {}

void GlobalPathPlannerRos::InitRos() {
  ROS_INFO("[GlobalPathPlannerRos]: Initialized");
  // load ros parameters

  // initialize parameters
  nh_.param<std::string>("control_command_topic", control_command_topic_, "/m545_path_follower_ros/command");
  nh_.param<std::string>("planning_service", planning_service_name_,
                         "/m545_planner_node/ompl_rs_planner_ros/planning_service");
  nh_.param<std::string>("current_state_service", current_state_service_name_,
                         "/m545_path_follower_ros/get_curr_se2_state");
}

void GlobalPathPlannerRos::Initialize(double dt) {
  // load parammeters
  // load path
  this->InitRos();
}

void GlobalPathPlannerRos::LoadPathFromFile(std::string filename) {}

bool GlobalPathPlannerRos::RequestPlan(geometry_msgs::Pose& start, geometry_msgs::Pose& goal) {
  ROS_INFO("Requesting plan from %f, %f, %f to %f, %f, %f", start.position.x, start.position.y, start.orientation.w,
           goal.position.x, goal.position.y, goal.orientation.w);

  bool service_call_result = true;
  std::string service_name = planning_service_name_;

  se2_navigation_msgs::RequestPathSrv::Request req;
  req.pathRequest.goalPose = goal;
  req.pathRequest.startingPose = start;
  se2_navigation_msgs::RequestPathSrv::Response res;
  service_call_result = CallService(req, res, service_name);

  if (!service_call_result) {
    ROS_ERROR("Planning service call failed");
  }

  return service_call_result;
}

void GlobalPathPlannerRos::RequestPose(geometry_msgs::Pose& pose) {
  se2_navigation_msgs::RequestCurrentStateSrv::Request req;
  se2_navigation_msgs::RequestCurrentStateSrv::Response res;
  CallService(req, res, current_state_service_name_);
  pose = res.pose;
}

bool GlobalPathPlannerRos::RequestPlanCurrentSegment(bool start_from_current_pose) {
  ROS_INFO("Requesting plan from current segment at index %d", current_segment_index_);
  // assert(current_segment_index_ < global_path_.size() - 1, "Current segment index is out of bounds");
  // catch exception if index is out of bounds
  geometry_msgs::Pose start;
  bool request_success;
  if (start_from_current_pose) {
    this->RequestPose(start);
  }
  try {
    start = global_path_.at(current_segment_index_);
    geometry_msgs::Pose goal = global_path_.at(current_segment_index_ + 1);
    ROS_INFO("Requesting plan from %f, %f, %f", start.position.x, start.position.y, start.orientation.w);
    // print start position
    ROS_INFO("start position from %f, %f, %f", start.position.x, start.position.y, start.orientation.w);
    // print start orientation
    ROS_INFO("start orientation to %f, %f, %f %f", start.orientation.x, start.orientation.y, start.orientation.z,
             start.orientation.w);
    // print goal position
    ROS_INFO("Goal position %f, %f, %f", goal.position.x, goal.position.y, goal.position.z);
    // print goal orientation
    ROS_INFO("Goal orientation %f, %f, %f %f", goal.orientation.x, goal.orientation.y, goal.orientation.z,
             goal.orientation.w);

    request_success = this->RequestPlan(start, goal);
    if (request_success) {
      current_segment_index_++;
    }
  } catch (std::exception& e) {
    ROS_ERROR("Current segment index is out of bounds");
    return false;
  }
  return request_success;
}

void GlobalPathPlannerRos::RequestStartTracking() {
  ROS_INFO("Requesting start tracking");
  se2_navigation_msgs::ControllerCommand command;
  command.command_ = se2_navigation_msgs::ControllerCommand::Command::StartTracking;

  se2_navigation_msgs::SendControllerCommandSrv::Request req;
  se2_navigation_msgs::SendControllerCommandSrv::Response res;

  req.command = se2_navigation_msgs::convert(command);
  std::string service_name = control_command_topic_;
  CallService(req, res, service_name);
}

void GlobalPathPlannerRos::RequestStopTracking() {
  ROS_INFO("Requesting stop tracking");
  se2_navigation_msgs::ControllerCommand command;
  command.command_ = se2_navigation_msgs::ControllerCommand::Command::StopTracking;

  se2_navigation_msgs::SendControllerCommandSrv::Request req;
  se2_navigation_msgs::SendControllerCommandSrv::Response res;

  req.command = se2_navigation_msgs::convert(command);
  std::string service_name = control_command_topic_;
  CallService(req, res, service_name);
}

void GlobalPathPlannerRos::LoadPath(std::vector<geometry_msgs::Pose>& path) {
  global_path_ = path;
  current_segment_index_ = 0;
}

const std::vector<geometry_msgs::Pose>& GlobalPathPlannerRos::GetPath() { return global_path_; }

void GlobalPathPlannerRos::LoadDummyPath() {
  geometry_msgs::Pose pose;
  float yaw = 0;
  pose.position.x = 0.0;
  pose.position.y = 0.0;

  auto q = m545_path_utils::eulerToQuaternion(0.0, 0.0, yaw);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  global_path_.push_back(pose);

  pose.position.x = 5.0;
  pose.position.y = 0.0;
  global_path_.push_back(pose);

  pose.position.x = 10.0;
  pose.position.y = 0;
  global_path_.push_back(pose);
}

bool GlobalPathPlannerRos::CompletedPath() { return current_segment_index_ >= global_path_.size() - 1; }

}  // namespace m545_coverage_planner_ros