#include <gtest/gtest.h>
#include "m545_coverage_planner_ros/GlobalPathRos.hpp"

TEST(GlobalPathTest, test_constructor)
{
  ros::NodeHandle nh;
  m545_coverage_planner_ros::GlobalPath path(nh);
}

TEST(GlobalPathTest, test_load_from_file){
  ros::NodeHandle nh;
  m545_coverage_planner_ros::GlobalPath path(nh);

}