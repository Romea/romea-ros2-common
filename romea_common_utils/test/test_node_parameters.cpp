// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <memory>
#include <string>
#include <vector>
#include <map>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/rclcpp.hpp"

// local
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/params/geodesy_parameters.hpp"
#include "romea_common_utils/params/eigen_parameters.hpp"
#include "romea_common_utils/params/algorithm_parameters.hpp"
#include "romea_common_utils/params/control_parameters.hpp"
#include "../test/test_helper.h"


class TestNodeParameters : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions no;

    no.arguments(
      {"--ros-args",
        "--params-file",
        std::string(TEST_DIR) + "/test_node_parameters.yaml"});

    node = std::make_shared<rclcpp::Node>("test_node_parameters", "ns", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestNodeParameters, getParameter) {
  romea::ros2::declare_parameter<std::string>(node, "foo");
  std::string value = romea::ros2::get_parameter<std::string>(node, "foo");
  EXPECT_STREQ(value.c_str(), "bar");
}

TEST_F(TestNodeParameters, getParameterDeclareDefault) {
  std::string default_value = "foo";
  romea::ros2::declare_parameter_with_default<std::string>(node, "bar", "foo");
  std::string value = romea::ros2::get_parameter<std::string>(node, "bar");
  EXPECT_STREQ(value.c_str(), default_value.c_str());
}

TEST_F(TestNodeParameters, getParameterOr) {
  std::string default_value = "foo";
  std::string value = romea::ros2::get_parameter_or<std::string>(node, "bar", default_value);
  EXPECT_STREQ(value.c_str(), default_value.c_str());
}

TEST_F(TestNodeParameters, getParameterInSubNamespace) {
  romea::ros2::declare_parameter<double>(node, "one", "two");
  double value = romea::ros2::get_parameter<double>(node, "one", "two");
  EXPECT_DOUBLE_EQ(value, 3.);
}


TEST_F(TestNodeParameters, loadVectorOfDouble) {
  std::vector<double> vector_of_double;
  romea::ros2::declare_vector_parameter<double>(node, "vector3d");
  EXPECT_NO_THROW(
    {vector_of_double = romea::ros2::get_vector_parameter<double>(node, "vector3d");});
  EXPECT_NEAR(vector_of_double[0], 2.3, 0.000001);
  EXPECT_NEAR(vector_of_double[1], 5.4, 0.000001);
  EXPECT_NEAR(vector_of_double[2], -8.9, 0.000001);
}


TEST_F(TestNodeParameters, loadVectorOfInt) {
  std::vector<int64_t> vector_of_int;
  romea::ros2::declare_vector_parameter<int64_t>(node, "vector3i");
  EXPECT_NO_THROW(
    {vector_of_int = romea::ros2::get_vector_parameter<int64_t>(node, "vector3i");});
  EXPECT_EQ(vector_of_int[0], 2);
  EXPECT_EQ(vector_of_int[1], -5);
  EXPECT_EQ(vector_of_int[2], 9);
}


TEST_F(TestNodeParameters, loadVectorOfIntWithDefaultDeclaration) {
  std::vector<int64_t> default_vector_of_int = {4, 6, -3};
  std::vector<int64_t> vector_of_int;
  romea::ros2::declare_vector_parameter_with_default<int64_t>(
    node, "vector_unset", default_vector_of_int);
  EXPECT_NO_THROW(
    {vector_of_int =
      romea::ros2::get_vector_parameter<int64_t>(node, "vector_unset");});
  EXPECT_EQ(vector_of_int[0], 4);
  EXPECT_EQ(vector_of_int[1], 6);
  EXPECT_EQ(vector_of_int[2], -3);
}

TEST_F(TestNodeParameters, loadEigenVector3d) {
  Eigen::Vector3d eigen_vector3d;
  romea::ros2::declare_eigen_vector_parameter<Eigen::Vector3d>(node, "vector3d");
  EXPECT_NO_THROW(
    {eigen_vector3d =
      romea::ros2::get_eigen_vector_parameter<Eigen::Vector3d>(node, "vector3d");});
  EXPECT_NEAR(eigen_vector3d.x(), 2.3, 0.000001);
  EXPECT_NEAR(eigen_vector3d.y(), 5.4, 0.000001);
  EXPECT_NEAR(eigen_vector3d.z(), -8.9, 0.000001);
}

TEST_F(TestNodeParameters, loadUnsetEigenVector3d) {
  Eigen::Vector3d default_eigen_vector3d(2, 3, 6);
  Eigen::Vector3d eigen_vector3d;
  romea::ros2::declare_eigen_vector_parameter_with_default<Eigen::Vector3d>(
    node, "unset_vector", default_eigen_vector3d);
  EXPECT_NO_THROW(
    {eigen_vector3d =
      romea::ros2::get_eigen_vector_parameter<Eigen::Vector3d>(node, "unset_vector");});
  EXPECT_NEAR(eigen_vector3d.x(), 2, 0.000001);
  EXPECT_NEAR(eigen_vector3d.y(), 3, 0.000001);
  EXPECT_NEAR(eigen_vector3d.z(), 6, 0.000001);
}

// TEST_F(TestNodeParameters, loadEigenVector3i) {

//    Eigen::Vector3i eigen_vector3i;
//    EXPECT_NO_THROW({eigen_vector3i=romea::loadEigenVector<Eigen::Vector3i>(*node,"vector3i");});
//    EXPECT_NEAR(eigen_vector3i.x(),2.3,0.000001);
//    EXPECT_NEAR(eigen_vector3i.y(),5.4,0.000001);
//    EXPECT_NEAR(eigen_vector3i.z(),-8.9,0.000001);
//}

TEST_F(TestNodeParameters, loadGeodeticCoordinates)
{
  romea::core::GeodeticCoordinates geodetic_coordinates;
  romea::ros2::declare_geodetic_coordinates_parameter(node, "geodetic");
  EXPECT_NO_THROW(
    {geodetic_coordinates = romea::ros2::get_geodetic_coordinates_parameter(node, "geodetic");});
  EXPECT_NEAR(geodetic_coordinates.latitude, 45.85207 / 180. * M_PI, 0.000001);
  EXPECT_NEAR(geodetic_coordinates.longitude, 3.16482 / 180. * M_PI, 0.000001);
  EXPECT_NEAR(geodetic_coordinates.altitude, 300.0, 0.000001);
}


TEST_F(TestNodeParameters, loadMapFloat)
{
  std::map<std::string, double> map;
  romea::ros2::declare_parameter<double>(node, "map.foo");
  romea::ros2::declare_parameter<double>(node, "map.bar");
  romea::ros2::declare_parameter<double>(node, "map.baz");
  EXPECT_NO_THROW({map = romea::ros2::get_parameters<double>(node, "map");});
  EXPECT_DOUBLE_EQ(map.at("foo"), 0.34);
  EXPECT_DOUBLE_EQ(map.at("bar"), -2.7);
  EXPECT_DOUBLE_EQ(map.at("baz"), 5.5);
}


TEST_F(TestNodeParameters, loadDebug)
{
  romea::ros2::declare_debug(node);
  EXPECT_TRUE(romea::ros2::get_debug(node));
}

TEST_F(TestNodeParameters, loadLogDirectory)
{
  romea::ros2::declare_log_directory(node);
  EXPECT_STREQ(romea::ros2::get_log_directory(node).c_str(), "/foo");
}

TEST_F(TestNodeParameters, loadLogFilename)
{
  romea::ros2::declare_debug(node);
  romea::ros2::declare_log_directory(node);
  EXPECT_STREQ(
    romea::ros2::get_log_filename(node, "bar").c_str(),
    "/foo/ns_test_node_parameters_bar_debug.csv");
}

TEST_F(TestNodeParameters, loadBaseFootprintFrameId)
{
  romea::ros2::declare_base_footprint_frame_id(node);
  EXPECT_STREQ(romea::ros2::get_base_footprint_frame_id(node).c_str(), "base_footprint1");
  romea::ros2::declare_base_footprint_frame_id(node, "bar");
  EXPECT_STREQ(romea::ros2::get_base_footprint_frame_id(node, "bar").c_str(), "base_footprint2");
  romea::ros2::declare_base_footprint_frame_id(node, "foo");
  EXPECT_STREQ(romea::ros2::get_base_footprint_frame_id(node, "foo").c_str(), "base_footprint");
}

TEST_F(TestNodeParameters, loadOdomFrameId)
{
  romea::ros2::declare_odom_frame_id(node);
  EXPECT_STREQ(romea::ros2::get_odom_frame_id(node).c_str(), "odom1");
  romea::ros2::declare_odom_frame_id(node, "bar");
  EXPECT_STREQ(romea::ros2::get_odom_frame_id(node, "bar").c_str(), "odom2");
  romea::ros2::declare_odom_frame_id(node, "foo");
  EXPECT_STREQ(romea::ros2::get_odom_frame_id(node, "foo").c_str(), "odom");
}

TEST_F(TestNodeParameters, loadMapFrameId)
{
  romea::ros2::declare_map_frame_id(node);
  EXPECT_STREQ(romea::ros2::get_map_frame_id(node).c_str(), "map1");
  romea::ros2::declare_map_frame_id(node, "bar");
  EXPECT_STREQ(romea::ros2::get_map_frame_id(node, "bar").c_str(), "map2");
  romea::ros2::declare_map_frame_id(node, "foo");
  EXPECT_STREQ(romea::ros2::get_map_frame_id(node, "foo").c_str(), "map");
}

TEST_F(TestNodeParameters, loadPublishRate)
{
  romea::ros2::declare_publish_rate(node);
  EXPECT_EQ(romea::ros2::get_publish_rate(node), 10);
  romea::ros2::declare_publish_rate(node, "bar");
  EXPECT_EQ(romea::ros2::get_publish_rate(node, "bar"), 20);
}

TEST_F(TestNodeParameters, loadPIDParameters)
{
  romea::ros2::declare_pid_parameters(node, "pid");
  auto pid_parameters = romea::ros2::get_pid_parameters(node, "pid");
  EXPECT_EQ(pid_parameters.kp, 1.0);
  EXPECT_EQ(pid_parameters.ki, 2.0);
  EXPECT_EQ(pid_parameters.kd, 3.0);
  EXPECT_EQ(pid_parameters.imin, 4.0);
  EXPECT_EQ(pid_parameters.imax, 5.0);
}

TEST(TestNodeParameterList, getValueFromParameterListFailed) {
  EXPECT_THROW(romea::ros2::get_parameter_value<double>({}, "foo"), std::runtime_error);
}

TEST(TestNodeParametersList, getValueFromParameterListOK) {
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("foo", 1.0));
  EXPECT_DOUBLE_EQ(romea::ros2::get_parameter_value<double>(parameters, "foo"), 1.0);
}

TEST(TestNodeParametersList, getValueOrFromParameterListOK) {
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("foo", true));
  EXPECT_TRUE(romea::ros2::get_parameter_value<bool>(parameters, "foo"));
}

TEST(TestNodeParametersList, getDefaultValueFromParameterList) {
  EXPECT_FALSE(romea::ros2::get_parameter_value_or<bool>({}, "foo", false));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
