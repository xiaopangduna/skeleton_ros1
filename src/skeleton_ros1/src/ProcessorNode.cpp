#include "skeleton_ros1/ProcessorNode.hpp"
#include <sstream>
#include <algorithm>

namespace skeleton_cpp {

ProcessorNode::ProcessorNode(const std::string& node_name) 
    : nh_(), rate_(10) {  // 10Hz
    
    // 订阅chatter主题，接收talker节点发布的消息
    sub_ = nh_.subscribe("chatter", 1000, &ProcessorNode::chatterCallback, this);
    
    // 发布processed_chatter主题，用于发布处理后的消息
    pub_ = nh_.advertise<std_msgs::String>("processed_chatter", 1000);
    
    ROS_INFO_STREAM("ProcessorNode initialized with name: " << node_name);
}

void ProcessorNode::run() {
    ROS_INFO("ProcessorNode running...");
    
    // 节点主循环
    while (ros::ok()) {
        ros::spinOnce();
        rate_.sleep();
    }
}

void ProcessorNode::chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Processing: [%s]", msg->data.c_str());
    
    // 创建处理后的消息
    std_msgs::String processed_msg;
    
    // 简单处理：将消息转换为大写并添加前缀
    std::string processed_data = "PROCESSED: " + msg->data;
    std::transform(processed_data.begin(), processed_data.end(), processed_data.begin(), ::toupper);
    
    processed_msg.data = processed_data;
    
    // 发布处理后的消息
    pub_.publish(processed_msg);
    
    ROS_INFO("Published: [%s]", processed_msg.data.c_str());
}

} // namespace skeleton_cpp