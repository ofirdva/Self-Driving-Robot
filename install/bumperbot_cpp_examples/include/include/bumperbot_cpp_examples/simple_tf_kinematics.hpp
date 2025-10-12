#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <memory>
#include <tf2_ros/buffer.h> //to get the transformation between the frames and interact with the tf library
#include <tf2_ros/transform_listener.h> //to get the transformation between the frames and interact with the tf library
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <bumperbot_msgs/srv/get_transform.hpp>  //use the service type generated fro gettransform.srv file
#include <tf2/LinearMath/Quaternion.h>

class SimpleTfKinematics : public rclcpp::Node
{
public:
    SimpleTfKinematics(const std::string &name);

  private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>dynamic_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

    rclcpp::Service<bumperbot_msgs::srv::GetTransform>::SharedPtr get_transform_srv_;

    rclcpp::TimerBase::SharedPtr timer_; 

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    
    double x_increment_;
    double last_x_;
    int rotations_counter_;
    tf2::Quaternion last_orientation_;
    tf2::Quaternion orientation_increment_;
    

    void timerCallback();


    // executes when the server get a new request
    bool getTransformCallback(const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> req,
                              const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> res);

  };

#endif