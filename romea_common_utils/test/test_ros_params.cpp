//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/rclcpp.hpp>

//romea
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/params/geodesy_parameters.hpp"
#include "romea_common_utils/params/eigen_parameters.hpp"


class TestRosParams : public ::testing::Test
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
     std::string(TEST_DIR)+"/test_ros_params.yaml"});

    node = std::make_shared<rclcpp::Node>("test_ros_params", no);
  }



  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestRosParams, getParameter) {
  romea::declare_parameter<std::string>(node,"foo");
  std::string value = romea::get_parameter<std::string>(node,"foo");
  EXPECT_STREQ(value.c_str(),"bar");
}

TEST_F(TestRosParams, getParameterDeclareDefault) {
  std::string default_value="foo";
  romea::declare_parameter_with_default<std::string>(node,"bar","foo");
  std::string value = romea::get_parameter<std::string>(node,"bar");
  EXPECT_STREQ(value.c_str(),default_value.c_str());
}

TEST_F(TestRosParams, getParameterOr) {
  std::string default_value="foo";
  std::string value = romea::get_parameter_or<std::string>(node,"bar",default_value);
  EXPECT_STREQ(value.c_str(),default_value.c_str());
}

TEST_F(TestRosParams, getParameterInSubNamespace) {
  romea::declare_parameter<double>(node,"one","two");
  double value = romea::get_parameter<double>(node,"one","two");
  EXPECT_DOUBLE_EQ(value,3.);
}


TEST_F(TestRosParams, loadVectorOfDouble) {

  std::vector<double> vector_of_double;
  romea::declare_vector_parameter<double>(node,"vector3d");
  EXPECT_NO_THROW({vector_of_double=romea::get_vector_parameter<double>(node,"vector3d");});
  EXPECT_NEAR(vector_of_double[0],2.3,0.000001);
  EXPECT_NEAR(vector_of_double[1],5.4,0.000001);
  EXPECT_NEAR(vector_of_double[2],-8.9,0.000001);
}

TEST_F(TestRosParams, loadVectorOfInt) {

  std::vector<long int> vector_of_int;
  romea::declare_vector_parameter<long int>(node,"vector3i");
  EXPECT_NO_THROW({vector_of_int=romea::get_vector_parameter<long int>(node,"vector3i");});
  EXPECT_EQ(vector_of_int[0],2);
  EXPECT_EQ(vector_of_int[1],-5);
  EXPECT_EQ(vector_of_int[2],9);
}


TEST_F(TestRosParams, loadEigenVector3d) {

  Eigen::Vector3d eigen_vector3d;
  romea::declare_eigen_vector_parameter<Eigen::Vector3d>(node,"vector3d");
  EXPECT_NO_THROW({eigen_vector3d=romea::get_eigen_vector_parameter<Eigen::Vector3d>(node,"vector3d");});
  EXPECT_NEAR(eigen_vector3d.x(),2.3,0.000001);
  EXPECT_NEAR(eigen_vector3d.y(),5.4,0.000001);
  EXPECT_NEAR(eigen_vector3d.z(),-8.9,0.000001);
}

//TEST_F(TestRosParams, loadEigenVector3i) {

//    Eigen::Vector3i eigen_vector3i;
//    EXPECT_NO_THROW({eigen_vector3i=romea::loadEigenVector<Eigen::Vector3i>(*node,"vector3i");});
//    EXPECT_NEAR(eigen_vector3i.x(),2.3,0.000001);
//    EXPECT_NEAR(eigen_vector3i.y(),5.4,0.000001);
//    EXPECT_NEAR(eigen_vector3i.z(),-8.9,0.000001);
//}

TEST_F(TestRosParams, loadGeodeticCoordinates)
{
  romea::GeodeticCoordinates geodetic_coordinates;
  romea::declare_geodetic_coordinates_parameter(node,"geodetic");
  EXPECT_NO_THROW({geodetic_coordinates=romea::get_geodetic_coordinates_parameter(node,"geodetic");});
  EXPECT_NEAR(geodetic_coordinates.latitude,45.85207/180.*M_PI,0.000001);
  EXPECT_NEAR(geodetic_coordinates.longitude,3.16482/180.*M_PI,0.000001);
  EXPECT_NEAR(geodetic_coordinates.altitude,300.0,0.000001);
}


TEST_F(TestRosParams, loadMapFloat)
{
  std::map<std::string,double> map;
  romea::declare_parameter<double>(node,"map.foo");
  romea::declare_parameter<double>(node,"map.bar");
  romea::declare_parameter<double>(node,"map.baz");
  EXPECT_NO_THROW({map=romea::get_parameters<double>(node,"map");});
  EXPECT_DOUBLE_EQ(map.at("foo"),0.34);
  EXPECT_DOUBLE_EQ(map.at("bar"),-2.7);
  EXPECT_DOUBLE_EQ(map.at("baz"),5.5);
}


//int main(int argc, char** argv)
//{
//  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "ros_param_test");

//  int ret = RUN_ALL_TESTS();
//  ros::shutdown();
//  return ret;
//}
