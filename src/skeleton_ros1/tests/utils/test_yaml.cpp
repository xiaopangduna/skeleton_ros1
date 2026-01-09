#include <filesystem>
#include <tuple>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "test_common/environment.hpp"
#include "skeleton_ros1/utils/yaml.hpp"

namespace fs = std::filesystem;

using namespace utils_yaml;

class UtilsYamlTest : public ::testing::Test
{
protected:
    static fs::path path_yaml;
    static YAML::Node test_data;

    static void SetUpTestSuite()
    {
        test_data = YAML::LoadFile(path_yaml);
    }

    static void TearDownTestSuite()
    {
    }
};

fs::path UtilsYamlTest::path_yaml = "examples/utils/test_yaml.yaml";
YAML::Node UtilsYamlTest::test_data;

TEST_F(UtilsYamlTest, ConvertYamlNodeToCvMatrix2NDouble)
{
    cv::Mat matrix = convertYamlNodeToCvMatrix2N<double>(UtilsYamlTest::test_data["matrix_double"]);

    ASSERT_EQ(matrix.rows, 2);
    ASSERT_EQ(matrix.cols, 3);
    ASSERT_EQ(matrix.type(), CV_64F);

    EXPECT_DOUBLE_EQ(matrix.at<double>(0, 0), 1.1);
    EXPECT_DOUBLE_EQ(matrix.at<double>(0, 1), 2.2);
    EXPECT_DOUBLE_EQ(matrix.at<double>(0, 2), 3.3);
    EXPECT_DOUBLE_EQ(matrix.at<double>(1, 0), 4.4);
    EXPECT_DOUBLE_EQ(matrix.at<double>(1, 1), 5.5);
    EXPECT_DOUBLE_EQ(matrix.at<double>(1, 2), 6.6);
}

TEST_F(UtilsYamlTest, ConvertYamlNodeToCvMatrix2NInt)
{
    cv::Mat matrix = convertYamlNodeToCvMatrix2N<int>(UtilsYamlTest::test_data["matrix_int"]);

    ASSERT_EQ(matrix.rows, 2);
    ASSERT_EQ(matrix.cols, 2);
    ASSERT_EQ(matrix.type(), CV_32S);

    EXPECT_EQ(matrix.at<int>(0, 0), 1);
    EXPECT_EQ(matrix.at<int>(0, 1), 2);
    EXPECT_EQ(matrix.at<int>(1, 0), 3);
    EXPECT_EQ(matrix.at<int>(1, 1), 4);
}

TEST_F(UtilsYamlTest, ConvertYamlNodeToVectorCvPoint2f)
{
    std::vector<cv::Point2f> points = convertYamlNodeToVectorCvPoint2N<cv::Point2f>(UtilsYamlTest::test_data["points_2d_float"]);

    ASSERT_EQ(points.size(), 3);

    EXPECT_FLOAT_EQ(points[0].x, 1.1f);
    EXPECT_FLOAT_EQ(points[0].y, 2.2f);
    EXPECT_FLOAT_EQ(points[1].x, 3.3f);
    EXPECT_FLOAT_EQ(points[1].y, 4.4f);
    EXPECT_FLOAT_EQ(points[2].x, 5.5f);
    EXPECT_FLOAT_EQ(points[2].y, 6.6f);
}

TEST_F(UtilsYamlTest, ConvertYamlNodeToVectorCvPoint2i)
{
    std::vector<cv::Point2i> points = convertYamlNodeToVectorCvPoint2N<cv::Point2i>(UtilsYamlTest::test_data["points_2d_int"]);

    ASSERT_EQ(points.size(), 3);

    EXPECT_EQ(points[0].x, 1);
    EXPECT_EQ(points[0].y, 2);
    EXPECT_EQ(points[1].x, 3);
    EXPECT_EQ(points[1].y, 4);
    EXPECT_EQ(points[2].x, 5);
    EXPECT_EQ(points[2].y, 6);
}

TEST_F(UtilsYamlTest, ConvertYamlNodeToVectorCvPoint3d)
{
    std::vector<cv::Point3d> points = convertYamlNodeToVectorCvPoint3N<cv::Point3d>(UtilsYamlTest::test_data["points_3d_double"]);

    ASSERT_EQ(points.size(), 3);

    EXPECT_DOUBLE_EQ(points[0].x, 1.1);
    EXPECT_DOUBLE_EQ(points[0].y, 2.2);
    EXPECT_DOUBLE_EQ(points[0].z, 3.3);
    EXPECT_DOUBLE_EQ(points[1].x, 4.4);
    EXPECT_DOUBLE_EQ(points[1].y, 5.5);
    EXPECT_DOUBLE_EQ(points[1].z, 6.6);
    EXPECT_DOUBLE_EQ(points[2].x, 7.7);
    EXPECT_DOUBLE_EQ(points[2].y, 8.8);
    EXPECT_DOUBLE_EQ(points[2].z, 9.9);
}

TEST_F(UtilsYamlTest, ConvertYamlNodeToVectorCvPoint3i)
{
    std::vector<cv::Point3i> points = convertYamlNodeToVectorCvPoint3N<cv::Point3i>(UtilsYamlTest::test_data["points_3d_int"]);

    ASSERT_EQ(points.size(), 3);

    EXPECT_EQ(points[0].x, 1);
    EXPECT_EQ(points[0].y, 2);
    EXPECT_EQ(points[0].z, 3);
    EXPECT_EQ(points[1].x, 4);
    EXPECT_EQ(points[1].y, 5);
    EXPECT_EQ(points[1].z, 6);
    EXPECT_EQ(points[2].x, 7);
    EXPECT_EQ(points[2].y, 8);
    EXPECT_EQ(points[2].z, 9);
}

// 新增逆过程函数的测试用例
TEST_F(UtilsYamlTest, ConvertCvMatrix2NToYamlNodeDouble)
{
    cv::Mat original_matrix = (cv::Mat_<double>(2, 3) << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6);
    
    YAML::Node yaml_node = convertCvMatrix2NToYamlNode<double>(original_matrix);
    
    cv::Mat converted_matrix = convertYamlNodeToCvMatrix2N<double>(yaml_node);
    
    ASSERT_EQ(converted_matrix.rows, 2);
    ASSERT_EQ(converted_matrix.cols, 3);
    ASSERT_EQ(converted_matrix.type(), CV_64F);

    EXPECT_DOUBLE_EQ(converted_matrix.at<double>(0, 0), 1.1);
    EXPECT_DOUBLE_EQ(converted_matrix.at<double>(0, 1), 2.2);
    EXPECT_DOUBLE_EQ(converted_matrix.at<double>(0, 2), 3.3);
    EXPECT_DOUBLE_EQ(converted_matrix.at<double>(1, 0), 4.4);
    EXPECT_DOUBLE_EQ(converted_matrix.at<double>(1, 1), 5.5);
    EXPECT_DOUBLE_EQ(converted_matrix.at<double>(1, 2), 6.6);
}

TEST_F(UtilsYamlTest, ConvertCvMatrix2NToYamlNodeInt)
{
    cv::Mat original_matrix = (cv::Mat_<int>(2, 2) << 1, 2, 3, 4);
    
    YAML::Node yaml_node = convertCvMatrix2NToYamlNode<int>(original_matrix);
    
    cv::Mat converted_matrix = convertYamlNodeToCvMatrix2N<int>(yaml_node);
    
    ASSERT_EQ(converted_matrix.rows, 2);
    ASSERT_EQ(converted_matrix.cols, 2);
    ASSERT_EQ(converted_matrix.type(), CV_32S);

    EXPECT_EQ(converted_matrix.at<int>(0, 0), 1);
    EXPECT_EQ(converted_matrix.at<int>(0, 1), 2);
    EXPECT_EQ(converted_matrix.at<int>(1, 0), 3);
    EXPECT_EQ(converted_matrix.at<int>(1, 1), 4);
}

TEST_F(UtilsYamlTest, ConvertVectorCvPoint2NToYamlNode)
{
    std::vector<cv::Point2f> original_points = {cv::Point2f(1.1f, 2.2f), cv::Point2f(3.3f, 4.4f), cv::Point2f(5.5f, 6.6f)};
    
    YAML::Node yaml_node = convertVectorCvPoint2NToYamlNode<cv::Point2f>(original_points);
    
    std::vector<cv::Point2f> converted_points = convertYamlNodeToVectorCvPoint2N<cv::Point2f>(yaml_node);
    
    ASSERT_EQ(converted_points.size(), 3);

    EXPECT_FLOAT_EQ(converted_points[0].x, 1.1f);
    EXPECT_FLOAT_EQ(converted_points[0].y, 2.2f);
    EXPECT_FLOAT_EQ(converted_points[1].x, 3.3f);
    EXPECT_FLOAT_EQ(converted_points[1].y, 4.4f);
    EXPECT_FLOAT_EQ(converted_points[2].x, 5.5f);
    EXPECT_FLOAT_EQ(converted_points[2].y, 6.6f);
}

TEST_F(UtilsYamlTest, ConvertVectorCvPoint3NToYamlNode)
{
    std::vector<cv::Point3d> original_points = {cv::Point3d(1.1, 2.2, 3.3), cv::Point3d(4.4, 5.5, 6.6), cv::Point3d(7.7, 8.8, 9.9)};
    
    YAML::Node yaml_node = convertVectorCvPoint3NToYamlNode<cv::Point3d>(original_points);
    
    std::vector<cv::Point3d> converted_points = convertYamlNodeToVectorCvPoint3N<cv::Point3d>(yaml_node);
    
    ASSERT_EQ(converted_points.size(), 3);

    EXPECT_DOUBLE_EQ(converted_points[0].x, 1.1);
    EXPECT_DOUBLE_EQ(converted_points[0].y, 2.2);
    EXPECT_DOUBLE_EQ(converted_points[0].z, 3.3);
    EXPECT_DOUBLE_EQ(converted_points[1].x, 4.4);
    EXPECT_DOUBLE_EQ(converted_points[1].y, 5.5);
    EXPECT_DOUBLE_EQ(converted_points[1].z, 6.6);
    EXPECT_DOUBLE_EQ(converted_points[2].x, 7.7);
    EXPECT_DOUBLE_EQ(converted_points[2].y, 8.8);
    EXPECT_DOUBLE_EQ(converted_points[2].z, 9.9);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}