//gtest
#include <gtest/gtest.h>

//ros
#include <rclcpp/rclcpp.hpp>

//romea
//#include "romea_common_utils/params/RosParam.hpp"
//#include "romea_common_utils/params/RosEigenParam.hpp"
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
        {
                        "--ros-args",
                        "--params-file","/home/jeanlaneurit/dev/romea_ros2/src/interfaces/core/romea_common/romea_common_utils/test/params.yaml"
                    });
        no.allow_undeclared_parameters(true);
        no.automatically_declare_parameters_from_overrides(true);

        node = std::make_shared<rclcpp::Node>("test_ros_params", no);
        node_parameters = std::make_shared<romea::NodeParameters>(node);
    }



    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<romea::NodeParameters> node_parameters;
};

TEST_F(TestRosParams, loadParamOr) {

  std::string default_value="toto";
  std::string value = node_parameters->loadParamOr("toto",default_value);
  std::cout <<  value << std::endl;
  EXPECT_STREQ(value.c_str(),default_value.c_str());

}


TEST_F(TestRosParams, loadVectorOfDouble) {

    std::vector<double> vector_of_double;
    EXPECT_NO_THROW({vector_of_double=node_parameters->loadVector<double>("vector3d");});
    EXPECT_NEAR(vector_of_double[0],2.3,0.000001);
    EXPECT_NEAR(vector_of_double[1],5.4,0.000001);
    EXPECT_NEAR(vector_of_double[2],-8.9,0.000001);
}

TEST_F(TestRosParams, loadVectorOfInt) {

    std::vector<long int> vector_of_int;
    EXPECT_NO_THROW({vector_of_int=node_parameters->loadVector<long int>("vector3i");});
    EXPECT_EQ(vector_of_int[0],2);
    EXPECT_EQ(vector_of_int[1],-5);
    EXPECT_EQ(vector_of_int[2],9);
}


TEST_F(TestRosParams, loadEigenVector3d) {

    Eigen::Vector3d eigen_vector3d;
    EXPECT_NO_THROW({eigen_vector3d=romea::loadEigenVector<Eigen::Vector3d>(*node_parameters,"vector3d");});
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
    EXPECT_NO_THROW({geodetic_coordinates=romea::loadGeodeticCoordinates(*node_parameters,"geodetic");});
    EXPECT_NEAR(geodetic_coordinates.latitude,45.85207/180.*M_PI,0.000001);
    EXPECT_NEAR(geodetic_coordinates.longitude,3.16482/180.*M_PI,0.000001);
    EXPECT_NEAR(geodetic_coordinates.altitude,300.0,0.000001);
}


TEST_F(TestRosParams, loadMapFloat)
{
    std::map<std::string,double> map;
    EXPECT_NO_THROW({map=node_parameters->loadMap<double>("map");});
    EXPECT_DOUBLE_EQ(map["foo"],0.34);
    EXPECT_DOUBLE_EQ(map["bar"],-2.7);
    EXPECT_DOUBLE_EQ(map["baz"],5.5);
}



//<test test-name="test_read_params"
//      pkg="romea_common_utils"
//      type="romea_common_utils_test_ros_params">
//    <rosparam param="vector3d">[2.3, 5.4, -8.9]</rosparam>
//    <rosparam param="vector3i">[2, -5, 9]</rosparam>
//    <rosparam param="geodetic">[45.85207, 3.16482, 300.0]</rosparam>
//    <rosparam param="map_float">
//       foo: 0.34
//       bar: -2.7
//       baz: 5.5
//    </rosparam>
//    <rosparam param="unavailable_map">
//       foo: 0.34
//       foo: -2.7
//       baz: 5.5
//    </rosparam>

//TEST(TestRosParams, loadVector3d)
//{
//  ros::NodeHandle private_nh("~");
//  Eigen::Vector3d vector3d;

//  EXPECT_NO_THROW({vector3d=romea::loadEigenVector<Eigen::Vector3d>(private_nh,"vector3d");});

//  EXPECT_NEAR(vector3d.x(),2.3,0.000001);
//  EXPECT_NEAR(vector3d.y(),5.4,0.000001);
//  EXPECT_NEAR(vector3d.z(),-8.9,0.000001);
//}

//TEST(TestRosParams, loadVector3i)
//{
//  ros::NodeHandle private_nh("~");
//  Eigen::Vector3d vector3d;

//  EXPECT_NO_THROW({vector3d= romea::loadEigenVector<Eigen::Vector3d>(private_nh,"vector3d");});

//  EXPECT_NEAR(vector3d.x(),2.3,0.000001);
//  EXPECT_NEAR(vector3d.y(),5.4,0.000001);
//  EXPECT_NEAR(vector3d.z(),-8.9,0.000001);
//}

//TEST(TestRosParams, loadGeodeticCoordinate)
//{
//  ros::NodeHandle private_nh("~");
//  romea::GeodeticCoordinates geodetic_coordinates;

//  EXPECT_NO_THROW({geodetic_coordinates=romea::loadGeodeticCoordinates(private_nh,"geodetic");});

//  EXPECT_NEAR(geodetic_coordinates.getLatitude(),45.85207/180.*M_PI,0.000001);
//  EXPECT_NEAR(geodetic_coordinates.getLongitude(),3.16482/180.*M_PI,0.000001);
//  EXPECT_NEAR(geodetic_coordinates.getAltitude(),300.0,0.000001);
//}

//TEST(TestRosParams, loadMapFloat)
//{
//  ros::NodeHandle private_nh("~");
//  std::map<std::string,double> map ;

//  EXPECT_NO_THROW({map=romea::loadMap<double>(private_nh,"map_float");});
//  EXPECT_DOUBLE_EQ(map["foo"],0.34);
//  EXPECT_DOUBLE_EQ(map["bar"],-2.7);
//  EXPECT_DOUBLE_EQ(map["baz"],5.5);
//}

//TEST(TestRosParams, loadUnavailableMap)
//{
//  ros::NodeHandle private_nh("~");
//  std::map<std::string,double> map ;
//  EXPECT_THROW({map=romea::loadMap<double>(private_nh,"unavailable_float");},std::runtime_error);
//}


//int main(int argc, char** argv)
//{
//  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "ros_param_test");

//  int ret = RUN_ALL_TESTS();
//  ros::shutdown();
//  return ret;
//}
