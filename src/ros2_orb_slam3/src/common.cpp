#include "ros2_orb_slam3/common.hpp"

StereoModeNode::StereoModeNode() : Node("orb_slam3_stereo")
{
    std::string voc_file = "/home/thundera/Workspace/ROS/mjc_drone_sim/src/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    std::string settings_file = "/home/thundera/Workspace/ROS/mjc_drone_sim/src/ros2_orb_slam3/params/sim.yaml";

    // Initialize in STEREO mode (No Inertial)
    pAgent = new ORB_SLAM3::System(voc_file, settings_file, ORB_SLAM3::System::STEREO, true);

    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/pose", 10);

    left_sub.subscribe(this, "/drone/camera/left/image_raw");
    right_sub.subscribe(this, "/drone/camera/right/image_raw");

    // Sync two images with same timestamp
    sync = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(left_sub, right_sub, 10);
    sync->registerCallback(std::bind(&StereoModeNode::GrabStereo, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Stereo SLAM Node Started");
}

void StereoModeNode::GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft, 
                               const sensor_msgs::msg::Image::ConstSharedPtr& msgRight)
{
    std::unique_lock<std::mutex> lock(m_mutex_process);
    
    cv_bridge::CvImagePtr cv_ptrL = cv_bridge::toCvCopy(msgLeft, "mono8");
    cv_bridge::CvImagePtr cv_ptrR = cv_bridge::toCvCopy(msgRight, "mono8");

    double t = msgLeft->header.stamp.sec + msgLeft->header.stamp.nanosec * 1e-9;
    
    // Pure Stereo tracking
    Sophus::SE3f Tcw = pAgent->TrackStereo(cv_ptrL->image, cv_ptrR->image, t);

    if (pAgent->GetTrackingState() == 2) {
        Sophus::SE3f Twc = Tcw.inverse();
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msgLeft->header.stamp;
        pose_msg.header.frame_id = "map";
        
        pose_msg.pose.position.x = Twc.translation().x();
        pose_msg.pose.position.y = Twc.translation().y();
        pose_msg.pose.position.z = Twc.translation().z();
        
        Eigen::Quaternionf q = Twc.unit_quaternion();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        
        pose_pub->publish(pose_msg);
    }
}
