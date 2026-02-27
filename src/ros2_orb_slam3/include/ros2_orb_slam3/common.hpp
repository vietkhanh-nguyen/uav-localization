#ifndef COMMON_HPP
#define COMMON_HPP

#include <iostream>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include <opencv2/core/core.hpp>
#include <System.h>

class StereoModeNode : public rclcpp::Node
{
public:
    StereoModeNode();
    ~StereoModeNode(){};

    void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft, 
                    const sensor_msgs::msg::Image::ConstSharedPtr& msgRight);

    ORB_SLAM3::System* pAgent;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    std::mutex m_mutex_process;
};

#endif
