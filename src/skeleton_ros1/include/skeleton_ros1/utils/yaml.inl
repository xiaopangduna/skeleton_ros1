#pragma once

/**
 * @file yaml.inl
 * @brief Yaml处理工具函数实现
 * @details 提供从Yaml节点转换为OpenCV数据结构的工具函数实现
 */

#include <type_traits>

namespace utils_yaml
{

    /// @brief 将Yaml节点转换为二维OpenCV矩阵数据的模板实现
    /// @tparam T 矩阵元素类型，支持double, int, float等基本类型
    /// @param node 包含矩阵信息的Yaml节点，应包含rows, cols, data字段
    /// @return 转换成功的cv::Mat对象
    /// @throws std::runtime_error 当Yaml节点格式不正确或数据不匹配时抛出异常
    template <typename T>
    cv::Mat convertYamlNodeToCvMatrix2N(const YAML::Node &node)
    {
        static_assert(
            std::is_same_v<T, double> ||
            std::is_same_v<T, float> ||
            std::is_same_v<T, int>,
            "T must be double, float, or int");

        // 检查必需的字段
        if (!node.IsMap() || !node["rows"] || !node["cols"] || !node["data"])
        {
            return cv::Mat();
        }

        int rows = node["rows"].as<int>();
        int cols = node["cols"].as<int>();

        cv::Mat matrix;
        const YAML::Node &data = node["data"];
        
        // 根据模板类型确定OpenCV矩阵类型
        if constexpr (std::is_same_v<T, double>) {
            matrix = cv::Mat::zeros(rows, cols, CV_64F);
            
            if (data.IsSequence() && static_cast<int>(data.size()) == rows * cols)
            {
                for (int i = 0; i < rows * cols; ++i)
                {
                    matrix.at<double>(i / cols, i % cols) = data[i].as<double>();
                }
            }
        }
        else if constexpr (std::is_same_v<T, float>) {
            matrix = cv::Mat::zeros(rows, cols, CV_32F);
            
            if (data.IsSequence() && static_cast<int>(data.size()) == rows * cols)
            {
                for (int i = 0; i < rows * cols; ++i)
                {
                    matrix.at<float>(i / cols, i % cols) = data[i].as<float>();
                }
            }
        }
        else if constexpr (std::is_same_v<T, int>) {
            matrix = cv::Mat::zeros(rows, cols, CV_32S);
            
            if (data.IsSequence() && static_cast<int>(data.size()) == rows * cols)
            {
                for (int i = 0; i < rows * cols; ++i)
                {
                    matrix.at<int>(i / cols, i % cols) = data[i].as<int>();
                }
            }
        }

        return matrix;
    }

    /// @brief 将二维OpenCV矩阵数据转换为Yaml节点
    /// @tparam T 矩阵元素类型，支持double, int, float等基本类型
    /// @param matrix 要转换的cv::Mat对象
    /// @return 转换成功的YAML::Node对象
    /// @throws std::runtime_error 当矩阵格式不正确时抛出异常
    template <typename T>
    YAML::Node convertCvMatrix2NToYamlNode(const cv::Mat &matrix)
    {
        static_assert(
            std::is_same_v<T, double> ||
            std::is_same_v<T, float> ||
            std::is_same_v<T, int>,
            "T must be double, float, or int");

        YAML::Node node;
        node["rows"] = matrix.rows;
        node["cols"] = matrix.cols;

        std::vector<T> data;
        data.reserve(matrix.rows * matrix.cols);

        if constexpr (std::is_same_v<T, double>) {
            for (int i = 0; i < matrix.rows; ++i) {
                for (int j = 0; j < matrix.cols; ++j) {
                    data.push_back(static_cast<T>(matrix.at<double>(i, j)));
                }
            }
        } else if constexpr (std::is_same_v<T, float>) {
            for (int i = 0; i < matrix.rows; ++i) {
                for (int j = 0; j < matrix.cols; ++j) {
                    data.push_back(static_cast<T>(matrix.at<float>(i, j)));
                }
            }
        } else if constexpr (std::is_same_v<T, int>) {
            for (int i = 0; i < matrix.rows; ++i) {
                for (int j = 0; j < matrix.cols; ++j) {
                    data.push_back(static_cast<T>(matrix.at<int>(i, j)));
                }
            }
        }

        node["data"] = data;
        return node;
    }

    template <typename Point2T>
    std::vector<Point2T> convertYamlNodeToVectorCvPoint2N(const YAML::Node &node)
    {
        // 编译期检查：确保是 2D 点
        static_assert(
            std::is_same_v<Point2T, cv::Point2f> ||
                std::is_same_v<Point2T, cv::Point2d> ||
                std::is_same_v<Point2T, cv::Point2i>,
            "Point2T must be cv::Point2f, cv::Point2d, or cv::Point2i");

        if (!node.IsDefined() || !node.IsSequence())
        {
            throw std::runtime_error("Yaml node is not a valid sequence.");
        }

        using T = typename Point2T::value_type;
        std::vector<Point2T> points;
        points.reserve(node.size());

        for (const auto &pt : node)
        {
            if (!pt.IsSequence() || pt.size() != 2)
            {
                throw std::runtime_error("Each 2D point must be [x, y].");
            }
            T x = pt[0].as<T>();
            T y = pt[1].as<T>();
            points.emplace_back(x, y);
        }
        return points;
    }

    /// @brief 将 OpenCV 2D 点向量转换为Yaml节点，支持 cv::Point2f, cv::Point2d, cv::Point2i
    /// @tparam Point2T 点类型，必须是 cv::Point2f, cv::Point2d, 或 cv::Point2i 之一
    /// @param points 点向量
    /// @return 转换成功的YAML::Node对象
    /// @throws std::runtime_error 当点向量格式不正确时抛出异常
    template <typename Point2T>
    YAML::Node convertVectorCvPoint2NToYamlNode(const std::vector<Point2T> &points)
    {
        static_assert(
            std::is_same_v<Point2T, cv::Point2f> ||
                std::is_same_v<Point2T, cv::Point2d> ||
                std::is_same_v<Point2T, cv::Point2i>,
            "Point2T must be cv::Point2f, cv::Point2d, or cv::Point2i");

        YAML::Node node = YAML::Load("[]");
        
        using T = typename Point2T::value_type;
        for (const auto &point : points) {
            YAML::Node pt_node;
            pt_node.push_back(point.x);
            pt_node.push_back(point.y);
            node.push_back(pt_node);
        }
        
        return node;
    }

    template <typename Point3T>
    std::vector<Point3T> convertYamlNodeToVectorCvPoint3N(const YAML::Node &node)
    {
        static_assert(
            std::is_same_v<Point3T, cv::Point3f> ||
                std::is_same_v<Point3T, cv::Point3d> ||
                std::is_same_v<Point3T, cv::Point3i>,
            "Point3T must be cv::Point3f, cv::Point3d, or cv::Point3i");

        if (!node.IsDefined() || !node.IsSequence())
        {
            throw std::runtime_error("Yaml node is not a valid sequence.");
        }

        using T = typename Point3T::value_type;
        std::vector<Point3T> points;
        points.reserve(node.size());

        for (const auto &pt : node)
        {
            if (!pt.IsSequence() || pt.size() != 3)
            {
                throw std::runtime_error("Each 3D point must be [x, y, z].");
            }
            T x = pt[0].as<T>();
            T y = pt[1].as<T>();
            T z = pt[2].as<T>();
            points.emplace_back(x, y, z);
        }
        return points;
    }

    /// @brief 将 OpenCV 3D 点向量转换为Yaml节点，支持 cv::Point3f, cv::Point3d, cv::Point3i
    /// @tparam Point3T 点类型，必须是 cv::Point3f, cv::Point3d, 或 cv::Point3i 之一
    /// @param points 点向量
    /// @return 转换成功的YAML::Node对象
    /// @throws std::runtime_error 当点向量格式不正确时抛出异常
    template <typename Point3T>
    YAML::Node convertVectorCvPoint3NToYamlNode(const std::vector<Point3T> &points)
    {
        static_assert(
            std::is_same_v<Point3T, cv::Point3f> ||
                std::is_same_v<Point3T, cv::Point3d> ||
                std::is_same_v<Point3T, cv::Point3i>,
            "Point3T must be cv::Point3f, cv::Point3d, or cv::Point3i");

        YAML::Node node = YAML::Load("[]");
        
        using T = typename Point3T::value_type;
        for (const auto &point : points) {
            YAML::Node pt_node;
            pt_node.push_back(point.x);
            pt_node.push_back(point.y);
            pt_node.push_back(point.z);
            node.push_back(pt_node);
        }
        
        return node;
    }
}