#include "m545_coverage_planner_ros/GlobalPathPlannerRos.hpp"

#include <se2_navigation_msgs/RequestPathSrv.h>
#include "se2_visualization_ros/visualization_helpers.hpp"
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <fstream>

#include "m545_path_utils/Helpers.hpp"
#include "se2_navigation_msgs/ControllerCommand.hpp"
#include "se2_navigation_msgs/RequestCurrentStateSrv.h"
#include "se2_navigation_msgs/SendControllerCommandSrv.h"

// include std::experimental
// #include <experimental/filesystem>

namespace m545_coverage_planner_ros {

GlobalPathPlannerRos::~GlobalPathPlannerRos() {}

void GlobalPathPlannerRos::initialize(double dt) {
  this->initRos();
}

void GlobalPathPlannerRos::initRos() {
  ROS_INFO("[GlobalPathPlannerRos]: Initialized");
  // load ros parameters

  // initialize parameters
  nh_.param<std::string>("control_command_topic", control_command_topic_, "/m545_path_follower_ros/command");
  nh_.param<std::string>("planning_service", planning_service_name_,
                         "/m545_planner_node/ompl_rs_planner_ros/planning_service");
  nh_.param<std::string>("current_state_service", current_state_service_name_,
                         "/m545_path_follower_ros/get_curr_se2_state");
  nh_.param<std::string>("path_topic", path_topic_, "/coverage/poses");
  globalPathSub_ = nh_.subscribe(path_topic_, 1, &GlobalPathPlannerRos::globalPathCallback, this);    
  // for visualization purposes
  globalPathPub_ = nh_.advertise<visualization_msgs::MarkerArray>("global_path_tracker/path_vis", 1);                   
}


void GlobalPathPlannerRos::globalPathCallback(const geometry_msgs::PoseArray& msg) {

  for (size_t i = 0; i < msg.poses.size(); i++) {
    geometry_msgs::Pose pose; 
    pose.position.x = msg.poses[i].position.x;
    pose.position.y = msg.poses[i].position.y;
    pose.position.z = msg.poses[i].position.z;
    pose.orientation.x = msg.poses[i].orientation.x;
    pose.orientation.y = msg.poses[i].orientation.y;
    pose.orientation.z = msg.poses[i].orientation.z;
    pose.orientation.w = msg.poses[i].orientation.w;
    globalPath_.push_back(pose);
  }
  publishPathPoses();
}


void GlobalPathPlannerRos::publishPathPoints() const {
  visualization_msgs::MarkerArray msg;
  int id = 0;
  double z = 0;

  msg.markers.reserve(globalPath_.size());
  for (const auto& pose : globalPath_) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "";
    marker.header.stamp = ros::Time::now();
    marker.id = id++;
    const double diameter = 1;
    Eigen::Vector3d pos(pose.position.x, pose.position.y, z);
    se2_visualization_ros::drawSphere(pos, 0.8 * se2_visualization_ros::Color::Green(), diameter, &marker);
    msg.markers.push_back(marker);
  }
  globalPathPub_.publish(msg);
}


void GlobalPathPlannerRos::publishPathPoses() const {
  visualization_msgs::MarkerArray msg;
  int id = 0;
  double z = 0.;

  msg.markers.reserve(globalPath_.size());
  for (const auto& pose : globalPath_) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // path frame 
    marker.ns = "";
    marker.header.stamp = ros::Time::now();
    marker.id = id++;
    Eigen::Vector3d pos(pose.position.x, pose.position.y, z);
    Eigen::Quaternion q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    se2_visualization_ros::drawArrowFromPositionOrientation(pos, q, se2_visualization_ros::Color::Yellow(),
                                                             0.8, 0.2, &marker);
    msg.markers.push_back(marker);
  }
  globalPathPub_.publish(msg);
}

bool GlobalPathPlannerRos::requestPlan(geometry_msgs::Pose& start, geometry_msgs::Pose& goal) {
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

void GlobalPathPlannerRos::requestPose(geometry_msgs::Pose& pose) {
  se2_navigation_msgs::RequestCurrentStateSrv::Request req;
  se2_navigation_msgs::RequestCurrentStateSrv::Response res;
  CallService(req, res, current_state_service_name_);
  pose = res.pose;
}

bool GlobalPathPlannerRos::requestPlanCurrentSegment(bool start_from_current_pose) {
  ROS_INFO("Requesting plan from current segment at index %d", current_segment_index_);

  geometry_msgs::Pose start;
  bool request_success;
  if (start_from_current_pose) {
    this->requestPose(start);
  }
  try {
    start = globalPath_.at(current_segment_index_);
    geometry_msgs::Pose goal = globalPath_.at(current_segment_index_ + 1);

    request_success = this->requestPlan(start, goal);
    if (request_success) {
      current_segment_index_++;
    }
  } catch (std::exception& e) {
    ROS_ERROR("Current segment index is out of bounds");
    return false;
  }
  return request_success;
}

void GlobalPathPlannerRos::requestStartTracking() {
  ROS_INFO("Requesting start tracking");
  se2_navigation_msgs::ControllerCommand command;
  command.command_ = se2_navigation_msgs::ControllerCommand::Command::StartTracking;

  se2_navigation_msgs::SendControllerCommandSrv::Request req;
  se2_navigation_msgs::SendControllerCommandSrv::Response res;

  req.command = se2_navigation_msgs::convert(command);
  std::string service_name = control_command_topic_;
  CallService(req, res, service_name);
}

void GlobalPathPlannerRos::requestStopTracking() {
  ROS_INFO("Requesting stop tracking");
  se2_navigation_msgs::ControllerCommand command;
  command.command_ = se2_navigation_msgs::ControllerCommand::Command::StopTracking;

  se2_navigation_msgs::SendControllerCommandSrv::Request req;
  se2_navigation_msgs::SendControllerCommandSrv::Response res;

  req.command = se2_navigation_msgs::convert(command);
  std::string service_name = control_command_topic_;
  CallService(req, res, service_name);
}

void GlobalPathPlannerRos::setPath(std::vector<geometry_msgs::Pose>& path) {
  globalPath_ = path;
  current_segment_index_ = 0;
}

const std::vector<geometry_msgs::Pose>& GlobalPathPlannerRos::getPath() { return globalPath_; }

bool GlobalPathPlannerRos::completedPath() { return current_segment_index_ >= globalPath_.size() - 1; }


// void GlobalPathPlannerRos::LoadDummyPath() {
//   geometry_msgs::Pose pose;
//   float yaw = 0;
//   pose.position.x = 0.0;
//   pose.position.y = 0.0;

//   auto q = m545_path_utils::eulerToQuaternion(0.0, 0.0, yaw);
//   pose.orientation.x = q.x();
//   pose.orientation.y = q.y();
//   pose.orientation.z = q.z();
//   pose.orientation.w = q.w();
//   globalPath_.push_back(pose);

//   // pose.position.x = 5.0;
//   // pose.position.y = 0.0;
//   // global_path_.push_back(pose);

//   // pose.position.x = 10.0;
//   // pose.position.y = 0;
//   // global_path_.push_back(pose);
//   pose.position.x = 10.0;
//   pose.position.y = 0;
//   float yaw_1 = 0;
//   auto q_1 = m545_path_utils::eulerToQuaternion(0.0, 0.0, yaw_1);
//   pose.orientation.x = q_1.x();
//   pose.orientation.y = q_1.y();
//   pose.orientation.z = q_1.z();
//   pose.orientation.w = q_1.w();
//   globalPath_.push_back(pose);

//   pose.position.x = 10.0;
//   pose.position.y = 30;
//   float yaw_2 = -M_PI;
//   auto q_2 = m545_path_utils::eulerToQuaternion(0.0, 0.0, yaw_2);
//   pose.orientation.x = q_2.x();
//   pose.orientation.y = q_2.y();
//   pose.orientation.z = q_2.z();
//   pose.orientation.w = q_2.w();
//   globalPath_.push_back(pose);
// }

// void GlobalPathPlannerRos::LoadPathFromCsv(std::string filename) {
//   // Load csv file from path
//   std::ifstream csv_file(filename);
//   // print curretn working directory with the stl library
//   // ROS_INFO_STREAM("Current working directory " << std::experimental::filesystem::current_path());
//   if (csv_file.fail()) {
//     ROS_ERROR("Failed to open file %s", filename.c_str());
//     return;
//   }
//   // find the number of lines of the csv file
//   // auto num_lines = std::count(std::istreambuf_iterator<char>(csv_file), std::istreambuf_iterator<char>(), '\n');
//   // ROS_INFO_STREAM("Number of lines in csv file: " << num_lines);

//   std::string line;
//   // vector of poses to be loaded from csv file: (x, y, yaw)
//   std::vector<geometry_msgs::Pose> path;
//   // check if file is opened correcly

//   // iterate through the lines
//   while (std::getline(csv_file, line)) {
//     std::istringstream iss(line);
//     // store the values x, y, yaw separated by a comma
//     std::vector<std::string> results(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());
//     // print the results
//     ROS_INFO_STREAM("x: " << results[0] << " y: " << results[1] << " yaw: " << results[2]);
//     geometry_msgs::Pose pose;
//     pose.position.x = std::stof(results[0]);
//     pose.position.y = std::stof(results[1]);

//     double yaw = std::stof(results[2]);
//     auto q = m545_path_utils::eulerToQuaternion(0.0, 0.0, yaw);
//     pose.orientation.x = q.x();
//     pose.orientation.y = q.y();
//     pose.orientation.z = q.z();
//     pose.orientation.w = q.w();

//     path.push_back(pose);
//   }
//   globalPath_ = path;
// }


}  // namespace m545_coverage_planner_ros