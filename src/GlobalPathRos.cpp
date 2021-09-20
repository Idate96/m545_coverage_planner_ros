#include "m545_coverage_planner_ros/GlobalPathRos.hpp"

#include <se2_navigation_msgs/RequestPathSrv.h>

#include <fstream>

#include "se2_navigation_msgs/ControllerCommand.hpp"
#include "se2_navigation_msgs/RequestCurrentStateSrv.h"
#include "se2_navigation_msgs/SendControllerCommandSrv.h"

namespace m545_coverage_planner_ros {

GlobalPathRos::GlobalPathRos(ros::NodeHandle& nh) : nh_(nh) {
  // initialize parameters
  nh_.param<std::string>("control_command_topic", control_command_topic_, "/m545_path_follower_ros/command");
  nh_.param<std::string>("planning_service", planning_service_name_,
                         "/m545_planner_node/ompl_rs_planner_ros/planning_service");
  nh_.param<std::string>("current_state_service", current_state_service_name_,
                         "/m545_path_follower_ros/get_curr_se2_state");
  trackingStatusSub_ =
      nh_.subscribe("/m545_path_follower_ros/tracking_status", 1, &GlobalPathRos::trackingStatusCallback, this);
}

GlobalPathRos::~GlobalPathRos() {}

void GlobalPathRos::loadPathFromFile(std::string filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    ROS_ERROR("Could not open file %s", filename.c_str());
    return;
  }
  // skip over the header line
  // iterate over the lines to extract the data into an eigen vector
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::vector<double> data;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
      data.push_back(std::stod(item));
    }
    if (data.size() != 2) {
      ROS_ERROR("Invalid data in line %s", line.c_str());
      continue;
    }
  }
}

void GlobalPathRos::requestPlan(geometry_msgs::Pose& start, geometry_msgs::Pose& goal) {
  ROS_INFO("Requesting plan from %f, %f, %f to %f, %f, %f", start.position.x, start.position.y, start.orientation.w,
           goal.position.x, goal.position.y, goal.orientation.w);

  bool serviceCallResult = true;
  std::string service_name = planning_service_name_;

  se2_navigation_msgs::RequestPathSrv::Request req;
  req.pathRequest.goalPose = goal;
  req.pathRequest.startingPose = start;
  se2_navigation_msgs::RequestPathSrv::Response res;
  serviceCallResult = callService(req, res, service_name);

  if (!serviceCallResult) {
    ROS_ERROR("Planning service call failed");
    return;
  }
}

void GlobalPathRos::requestPose(geometry_msgs::Pose& pose) {
  se2_navigation_msgs::RequestCurrentStateSrv::Request req;
  se2_navigation_msgs::RequestCurrentStateSrv::Response res;
  bool serviceCallResult = callService(req, res, current_state_service_name_);
  pose = res.pose;
}

void GlobalPathRos::requestPlanCurrentSegment(bool startFromCurrentPose) {
  ROS_INFO("Requesting plan from current segment");
  // assert(currentSegmentIndex_ < globalPath_.size() - 1, "Current segment index is out of bounds");
  // catch exception if index is out of bounds
  geometry_msgs::Pose start;
  if (startFromCurrentPose) {
    requestPose(start);
  }
  try {
    start = globalPath_[currentSegmentIndex_];
    geometry_msgs::Pose goal = globalPath_[currentSegmentIndex_ + 1];
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

    requestPlan(start, goal);
  } catch (std::exception& e) {
    ROS_ERROR("Current segment index is out of bounds");
  }
}

void GlobalPathRos::requestStartTracking() {
  ROS_INFO("Requesting start tracking");
  se2_navigation_msgs::ControllerCommand command;
  command.command_ = se2_navigation_msgs::ControllerCommand::Command::StartTracking;

  se2_navigation_msgs::SendControllerCommandSrv::Request req;
  se2_navigation_msgs::SendControllerCommandSrv::Response res;

  req.command = se2_navigation_msgs::convert(command);
  std::string service_name = control_command_topic_;
  callService(req, res, service_name);
}

void GlobalPathRos::requestStopTracking() {
  ROS_INFO("Requesting stop tracking");
  se2_navigation_msgs::ControllerCommand command;
  command.command_ = se2_navigation_msgs::ControllerCommand::Command::StopTracking;

  se2_navigation_msgs::SendControllerCommandSrv::Request req;
  se2_navigation_msgs::SendControllerCommandSrv::Response res;

  req.command = se2_navigation_msgs::convert(command);
  std::string service_name = control_command_topic_;
  bool calledService = callService(req, res, service_name);
}

void GlobalPathRos::loadPath(std::vector<geometry_msgs::Pose>& path) {
  globalPath_ = path;
  currentSegmentIndex_ = 0;
}

void GlobalPathRos::trackingStatusCallback(const m545_planner_msgs::PathFollowerTrackingStatusRos& msg) {
  // when tracking status switches to true, request a new plan

  if (!!msg.status != prevTracking_) {
    ROS_INFO("Tracking status changed to %d", msg.status);
    ROS_INFO("Is path completed : %d", completedPath());
    if (!completedPath()) {
      // current state should match last waypoint
      requestPlanCurrentSegment(true);
      requestStartTracking();
      currentSegmentIndex_++;
    } else {
      requestStopTracking();
    }
    prevTracking_ = msg.status;
  }
  // ROS_INFO("Tracking status global path: %d", msg.status);
}

bool GlobalPathRos::completedPath() { return currentSegmentIndex_ > globalPath_.size() - 1; }

}  // namespace m545_coverage_planner_ros