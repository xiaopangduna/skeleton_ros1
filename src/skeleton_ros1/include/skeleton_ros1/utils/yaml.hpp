#pragma once

/**
 * @file yaml.hpp
 * @brief Yaml处理工具函数集合
 * @details 提供从Yaml节点转换为OpenCV数据结构的工具函数
 */

#include <vector>
#include <string>

#include <type_traits>
#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

namespace utils_yaml
{

    /// @brief 将Yaml节点转换为二维OpenCV矩阵数据
    /// @tparam T 矩阵元素类型，支持double, int, float等基本类型
    /// @param node 包含矩阵信息的Yaml节点，应包含rows, cols, data字段
    /// @return 转换成功的cv::Mat对象
    /// @throws std::runtime_error 当Yaml节点格式不正确或数据不匹配时抛出异常
    ///
    /// 使用示例:
    /// @code
    /// YAML::Node config = YAML::LoadFile("matrix.yaml");
    /// auto matrix_double = utils_yaml::convertYamlNodeToCvMatrix2N<double>(config["matrix_double"]);
    /// auto matrix_int = utils_yaml::convertYamlNodeToCvMatrix2N<int>(config["matrix_int"]);
    /// @endcode
    template <typename T>
    cv::Mat convertYamlNodeToCvMatrix2N(const YAML::Node &node);

    /// @brief 将二维OpenCV矩阵数据转换为Yaml节点
    /// @tparam T 矩阵元素类型，支持double, int, float等基本类型
    /// @param matrix 要转换的cv::Mat对象
    /// @return 转换成功的YAML::Node对象
    /// @throws std::runtime_error 当矩阵格式不正确时抛出异常
    ///
    /// 使用示例:
    /// @code
    /// cv::Mat mat = cv::Mat::eye(3, 3, CV_64F);
    /// auto yaml_node = utils_yaml::convertCvMatrix2NToYamlNode<double>(mat);
    /// @endcode
    template <typename T>
    YAML::Node convertCvMatrix2NToYamlNode(const cv::Mat &matrix);

    /// @brief 将Yaml节点转换为 OpenCV 2D 点向量，支持 cv::Point2f, cv::Point2d, cv::Point2i
    /// @tparam Point2T 点类型，必须是 cv::Point2f, cv::Point2d, 或 cv::Point2i 之一
    /// @param node Yaml节点，应为序列类型，每个元素为包含[x,y]的序列
    /// @return 点向量
    /// @throws std::runtime_error 当Yaml节点格式不正确时抛出异常
    ///
    /// 使用示例:
    /// @code
    /// YAML::Node config = YAML::LoadFile("points.yaml");
    /// auto points2f = utils_yaml::convertYamlNodeToVectorCvPoint2N<cv::Point2f>(config["points_2d"]);
    /// for (const auto& point : points2f) {
    ///     std::cout << "Point: (" << point.x << ", " << point.y << ")" << std::endl;
    /// }
    /// @endcode
    template <typename Point2T>
    std::vector<Point2T> convertYamlNodeToVectorCvPoint2N(const YAML::Node &node);

    /// @brief 将 OpenCV 2D 点向量转换为Yaml节点，支持 cv::Point2f, cv::Point2d, cv::Point2i
    /// @tparam Point2T 点类型，必须是 cv::Point2f, cv::Point2d, 或 cv::Point2i 之一
    /// @param points 点向量
    /// @return 转换成功的YAML::Node对象
    /// @throws std::runtime_error 当点向量格式不正确时抛出异常
    ///
    /// 使用示例:
    /// @code
    /// std::vector<cv::Point2f> points {{1.0f, 2.0f}, {3.0f, 4.0f}};
    /// auto yaml_node = utils_yaml::convertVectorCvPoint2NToYamlNode<cv::Point2f>(points);
    /// @endcode
    template <typename Point2T>
    YAML::Node convertVectorCvPoint2NToYamlNode(const std::vector<Point2T> &points);

    /// @brief 将Yaml节点转换为 OpenCV 3D 点向量，支持 cv::Point3f, cv::Point3d, cv::Point3i
    /// @tparam Point3T 点类型，必须是 cv::Point3f, cv::Point3d, 或 cv::Point3i 之一
    /// @param node Yaml节点，应为序列类型，每个元素为包含[x,y,z]的序列
    /// @return 点向量
    /// @throws std::runtime_error 当Yaml节点格式不正确时抛出异常
    ///
    /// 使用示例:
    /// @code
    /// YAML::Node config = YAML::LoadFile("points.yaml");
    /// auto points3d = utils_yaml::convertYamlNodeToVectorCvPoint3N<cv::Point3d>(config["points_3d"]);
    /// for (const auto& point : points3d) {
    ///     std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    /// }
    /// @endcode
    template <typename Point3T>
    std::vector<Point3T> convertYamlNodeToVectorCvPoint3N(const YAML::Node &node);

    /// @brief 将 OpenCV 3D 点向量转换为Yaml节点，支持 cv::Point3f, cv::Point3d, cv::Point3i
    /// @tparam Point3T 点类型，必须是 cv::Point3f, cv::Point3d, 或 cv::Point3i 之一
    /// @param points 点向量
    /// @return 转换成功的YAML::Node对象
    /// @throws std::runtime_error 当点向量格式不正确时抛出异常
    ///
    /// 使用示例:
    /// @code
    /// std::vector<cv::Point3d> points {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    /// auto yaml_node = utils_yaml::convertVectorCvPoint3NToYamlNode<cv::Point3d>(points);
    /// @endcode
    template <typename Point3T>
    YAML::Node convertVectorCvPoint3NToYamlNode(const std::vector<Point3T> &points);
}

#include "yaml.inl"