#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <mujoco/mujoco.h>
#include <memory>
#include <string>

class OdomPublisher {
public:
    OdomPublisher(rclcpp::Node::SharedPtr node, 
                  const std::string& base_body_name = "base",
                  const std::string& odom_topic = "/odom")
        : node_(node), base_body_name_(base_body_name), enabled_(false) {
        
        // Declare parameters
        node_->declare_parameter("odom.enabled", true);
        node_->declare_parameter("odom.frame_id", "odom");
        node_->declare_parameter("odom.child_frame_id", "base_link");
        node_->declare_parameter("odom.publish_tf", true);
        node_->declare_parameter("odom.world_frame_id", "world");
        
        // Get parameters
        enabled_ = node_->get_parameter("odom.enabled").as_bool();
        odom_frame_id_ = node_->get_parameter("odom.frame_id").as_string();
        child_frame_id_ = node_->get_parameter("odom.child_frame_id").as_string();
        publish_tf_ = node_->get_parameter("odom.publish_tf").as_bool();
        world_frame_id_ = node_->get_parameter("odom.world_frame_id").as_string();
        
        if (!enabled_) {
            RCLCPP_INFO(node_->get_logger(), "Odometry publisher is disabled");
            return;
        }
        
        // Create publisher
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
        
        // Create TF broadcaster
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
        }
        
        RCLCPP_INFO(node_->get_logger(), "Odometry publisher initialized");
        RCLCPP_INFO(node_->get_logger(), "  Topic: %s", odom_topic.c_str());
        RCLCPP_INFO(node_->get_logger(), "  TF: %s -> %s -> %s", 
                    world_frame_id_.c_str(), odom_frame_id_.c_str(), child_frame_id_.c_str());
    }
    
    ~OdomPublisher() = default;
    
    void initialize(const mjModel* m, const mjData* d) {
        if (!enabled_) return;
        
        // Try to find base body with multiple common names
        const char* base_names[] = {
            base_body_name_.c_str(),
            "base",
            "pelvis", 
            "trunk",
            "torso",
            "base_link",
            "root",
            nullptr
        };
        
        for (int i = 0; base_names[i] != nullptr; i++) {
            base_body_id_ = mj_name2id(m, mjOBJ_BODY, base_names[i]);
            if (base_body_id_ >= 0) {
                RCLCPP_INFO(node_->get_logger(), 
                           "Found base body: '%s' (id=%d)", 
                           base_names[i], base_body_id_);
                break;
            }
        }
        
        // If still not found, use first body after world (id=0)
        if (base_body_id_ < 0 && m->nbody > 1) {
            base_body_id_ = 1;  // First body after world
            RCLCPP_WARN(node_->get_logger(), 
                       "Using default base body: '%s' (id=1)", 
                       mj_id2name(m, mjOBJ_BODY, base_body_id_));
        }
        
        if (base_body_id_ < 0) {
            RCLCPP_ERROR(node_->get_logger(), 
                        "Could not find any base body, odometry disabled");
            enabled_ = false;
            return;
        }
        
        RCLCPP_INFO(node_->get_logger(), 
                   "Odometry tracking body: '%s' (id=%d)", 
                   mj_id2name(m, mjOBJ_BODY, base_body_id_), base_body_id_);
        
        // Initialize previous state
        last_update_time_ = node_->now();
    }
    
    void publish(const mjModel* m, const mjData* d) {
        if (!enabled_ || base_body_id_ < 0) return;
        
        auto current_time = node_->now();
        
        // Get body position and orientation
        mjtNum* pos = d->xpos + 3 * base_body_id_;
        mjtNum* quat = d->xquat + 4 * base_body_id_;
        
        // Get body velocity (computed from xpos difference)
        mjtNum* cvel = d->cvel + 6 * base_body_id_;  // 6-DOF velocity
        
        // Create odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = child_frame_id_;
        
        // Position
        odom_msg.pose.pose.position.x = pos[0];
        odom_msg.pose.pose.position.y = pos[1];
        odom_msg.pose.pose.position.z = pos[2];
        
        // Orientation (MuJoCo uses wxyz, ROS uses xyzw)
        odom_msg.pose.pose.orientation.w = quat[0];
        odom_msg.pose.pose.orientation.x = quat[1];
        odom_msg.pose.pose.orientation.y = quat[2];
        odom_msg.pose.pose.orientation.z = quat[3];
        
        // Linear velocity (in body frame, convert to world frame)
        odom_msg.twist.twist.linear.x = cvel[3];
        odom_msg.twist.twist.linear.y = cvel[4];
        odom_msg.twist.twist.linear.z = cvel[5];
        
        // Angular velocity
        odom_msg.twist.twist.angular.x = cvel[0];
        odom_msg.twist.twist.angular.y = cvel[1];
        odom_msg.twist.twist.angular.z = cvel[2];
        
        // Covariance (simplified, could be improved)
        for (int i = 0; i < 36; i++) {
            odom_msg.pose.covariance[i] = 0.0;
            odom_msg.twist.covariance[i] = 0.0;
        }
        // Diagonal values (small uncertainty for simulation)
        odom_msg.pose.covariance[0] = 0.001;   // x
        odom_msg.pose.covariance[7] = 0.001;   // y
        odom_msg.pose.covariance[14] = 0.001;  // z
        odom_msg.pose.covariance[21] = 0.001;  // roll
        odom_msg.pose.covariance[28] = 0.001;  // pitch
        odom_msg.pose.covariance[35] = 0.001;  // yaw
        
        odom_msg.twist.covariance[0] = 0.001;
        odom_msg.twist.covariance[7] = 0.001;
        odom_msg.twist.covariance[14] = 0.001;
        odom_msg.twist.covariance[21] = 0.001;
        odom_msg.twist.covariance[28] = 0.001;
        odom_msg.twist.covariance[35] = 0.001;
        
        // Publish odometry
        odom_pub_->publish(odom_msg);
        
        // Publish TF if enabled
        if (publish_tf_ && tf_broadcaster_) {
            // 1. Publish world -> odom (identity transform, odom is fixed in world)
            geometry_msgs::msg::TransformStamped world_to_odom;
            world_to_odom.header.stamp = current_time;
            world_to_odom.header.frame_id = world_frame_id_;
            world_to_odom.child_frame_id = odom_frame_id_;
            
            world_to_odom.transform.translation.x = 0.0;
            world_to_odom.transform.translation.y = 0.0;
            world_to_odom.transform.translation.z = 0.0;
            
            world_to_odom.transform.rotation.w = 1.0;
            world_to_odom.transform.rotation.x = 0.0;
            world_to_odom.transform.rotation.y = 0.0;
            world_to_odom.transform.rotation.z = 0.0;
            
            tf_broadcaster_->sendTransform(world_to_odom);
            
            // 2. Publish odom -> base_link (robot pose)
            geometry_msgs::msg::TransformStamped odom_to_base;
            odom_to_base.header.stamp = current_time;
            odom_to_base.header.frame_id = odom_frame_id_;
            odom_to_base.child_frame_id = child_frame_id_;
            
            odom_to_base.transform.translation.x = pos[0];
            odom_to_base.transform.translation.y = pos[1];
            odom_to_base.transform.translation.z = pos[2];
            
            odom_to_base.transform.rotation.w = quat[0];
            odom_to_base.transform.rotation.x = quat[1];
            odom_to_base.transform.rotation.y = quat[2];
            odom_to_base.transform.rotation.z = quat[3];
            
            tf_broadcaster_->sendTransform(odom_to_base);
        }
        
        last_update_time_ = current_time;
    }
    
    bool isEnabled() const { return enabled_; }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    std::string base_body_name_;
    std::string odom_frame_id_;
    std::string child_frame_id_;
    std::string world_frame_id_;
    bool enabled_;
    bool publish_tf_;
    
    int base_body_id_ = -1;
    rclcpp::Time last_update_time_;
};
