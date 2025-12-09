#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace skeleton_cpp {

class ProcessorNode {
public:
    /**
     * @brief 构造函数
     * @param node_name 节点名称
     */
    ProcessorNode(const std::string& node_name);
    
    /**
     * @brief 运行节点主循环
     */
    void run();
    
private:
    /**
     * @brief 处理接收到的消息
     * @param msg 接收到的消息
     */
    void chatterCallback(const std_msgs::String::ConstPtr& msg);
    
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Rate rate_;
};

} // namespace skeleton_cpp