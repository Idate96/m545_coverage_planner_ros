#include <gtest/gtest.h>

#include "m545_coverage_planner_ros/GlobalPathPlannerRos.hpp"

TEST(GlobalPathTest, test_constructor) {
  m545_coverage_planner_ros::GlobalPathPlannerRos global_path_planner;
  global_path_planner.initialize(0.0);
}

TEST(GlobalPathTest, laod_dummy_path) {
  m545_coverage_planner_ros::GlobalPathPlannerRos global_path_planner;
  global_path_planner.initialize(0.0);
  global_path_planner.loadDummyPath();

  auto path = global_path_planner.getPath();

  ASSERT_EQ(path.size(), 3);
  ASSERT_EQ(path[0].position.x, 0.0);
  ASSERT_EQ(path[0].position.y, 0.0);
  ASSERT_EQ(path[1].position.x, 1.0);
  ASSERT_EQ(path[1].position.y, 0.0);
  ASSERT_EQ(path[2].position.x, 1.0);
  ASSERT_EQ(path[2].position.y, 1.0);
}

TEST(GlobalPathTest, test_load_from_file) {}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
