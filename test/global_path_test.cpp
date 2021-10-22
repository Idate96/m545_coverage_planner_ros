#include <gtest/gtest.h>

// #include <experimental/filesystem>

#include "m545_coverage_planner_ros/GlobalPathPlannerRos.hpp"
#include "m545_path_utils/Helpers.hpp"

/*!
 * \brief convert a ros quaternion to an eigen euler angle
 */
Eigen::Vector3d rosQuat2eigenEuler(const geometry_msgs::Quaternion &quat) {
  // convert to eigen quaternion
  Eigen::Quaterniond quat_eigen(quat.w, quat.x, quat.y, quat.z);
  // convert to eigen euler rpy
  Eigen::Vector3d euler = quat_eigen.toRotationMatrix().eulerAngles(2, 1, 0);
  return euler;
}

TEST(GlobalPathTest, test_constructor) {
  m545_coverage_planner_ros::GlobalPathPlannerRos global_path_planner;
  global_path_planner.Initialize(0.0);
}

TEST(GlobalPathTest, laod_dummy_path) {
  m545_coverage_planner_ros::GlobalPathPlannerRos global_path_planner;
  global_path_planner.Initialize(0.0);
  global_path_planner.LoadDummyPath();

  auto path = global_path_planner.GetPath();

  ASSERT_EQ(path.size(), 2);
  // ASSERT_EQ(path[0].position.x, 0.0);
  // ASSERT_EQ(path[0].position.y, 0.0);
  // ASSERT_EQ(path[1].position.x, 1.0);
  // ASSERT_EQ(path[1].position.y, 0.0);
  // ASSERT_EQ(path[2].position.x, 1.0);
  // ASSERT_EQ(path[2].position.y, 1.0);
}

TEST(GlobalPathTest, test_load_from_csvfile) {
  m545_coverage_planner_ros::GlobalPathPlannerRos global_path_planner;
  global_path_planner.Initialize(0.0);
  // get current directory
  // std::string current_dir = std::::filesystem::current_path();
  std::string file_name = "../data/test_pose.csv";
  std::string file_path = current_dir + "/" + file_name;
  global_path_planner.LoadPathFromCsv(file_path);

  ASSERT_EQ(global_path_planner.GetPath().size(), 3);
  ASSERT_EQ(global_path_planner.GetPath()[0].position.x, 0.0);
  ASSERT_EQ(global_path_planner.GetPath()[0].position.y, 0.0);
  ASSERT_EQ(global_path_planner.GetPath()[1].position.x, 1.0);
  ASSERT_EQ(global_path_planner.GetPath()[1].position.y, 0.0);
  ASSERT_EQ(global_path_planner.GetPath()[2].position.x, 10.0);
  ASSERT_EQ(global_path_planner.GetPath()[2].position.y, 5.0);
  // assert that the orientation is correct
  // first change the type of the quaternion

  std::vector<double> expected_orientation = {0.0, 0.0, 2.0};

  // assert the orientation is correct for the rest of the path
  for (auto i = 0; i < global_path_planner.GetPath().size(); i++) {
    auto euler = rosQuat2eigenEuler(global_path_planner.GetPath()[i].orientation);
    ASSERT_NEAR(euler[0], expected_orientation[0], 0.001);
    ASSERT_NEAR(euler[1], expected_orientation[1], 0.001);
    ASSERT_NEAR(euler[2], expected_orientation[2], 0.001);
  }
}

TEST(GlobathPathTest, test_quat_conversion) {
  // eigen rpy angles
  Eigen::Vector3d euler_angles(0.0, 1.0, 1.0);
  // convert to quaternion
  auto quat_eigen = m545_path_utils::eulerToQuaternion(euler_angles[0], euler_angles[1], euler_angles[2]);
  // convert back to rpy
  auto euler_angles_back = m545_path_utils::quaternionToEuler(quat_eigen);
  // assert that the conversion is correct
  ASSERT_EQ(euler_angles_back.x(), euler_angles.x());
  ASSERT_EQ(euler_angles_back.y(), euler_angles.y());
  ASSERT_EQ(euler_angles_back.z(), euler_angles.z());

  // ros rpy angles
  geometry_msgs::Quaternion quat_ros;
  quat_ros.x = 0.0;
  quat_ros.y = 1.0;
  quat_ros.z = 1.0;
  quat_ros.w = 0.0;
  // convert to eigen quaternion
  auto quat_eigen_ros = m545_path_utils::rosQuat2eigenQuat(quat_ros);
  // convert back to ros quaternion
  auto quat_ros_back = m545_path_utils::eigenQuat2rosQuat(quat_eigen_ros);
  // assert that the conversion is correct
  ASSERT_EQ(quat_ros_back.x, quat_ros.x);
  ASSERT_EQ(quat_ros_back.y, quat_ros.y);
  ASSERT_EQ(quat_ros_back.z, quat_ros.z);
  ASSERT_EQ(quat_ros_back.w, quat_ros.w);

  // eigen quat to euler rpy
  auto euler_angles_eigen = m545_path_utils::quaternionToEuler(quat_eigen);

  // ros quat to eigen rpy
  auto euler_angles_ros = rosQuat2eigenEuler(quat_ros);
  // assert that the conversion is correct
  ASSERT_EQ(euler_angles_eigen.x(), euler_angles_ros.x());
  ASSERT_EQ(euler_angles_eigen.y(), euler_angles_ros.y());
  ASSERT_EQ(euler_angles_eigen.z(), euler_angles_ros.z());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
