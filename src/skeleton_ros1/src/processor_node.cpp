#include "skeleton_ros1/ProcessorNode.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "cpp_processor");
    
    // 创建ProcessorNode实例
    skeleton_cpp::ProcessorNode processor_node("cpp_processor");
    
    // 运行节点
    processor_node.run();
    
    return 0;
}