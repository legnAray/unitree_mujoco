#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mujoco/mujoco.h>
#include <memory>
#include <string>
#include <mutex>

class OdomSubscriber {
public:
    OdomSubscriber(rclcpp::Node::SharedPtr node, 
                   const std::string& base_body_name = "base",
                   bool enabled = false,
                   const std::string& mode = "odom",
                   const std::string& topic = "/external_odom",
                   const std::string& tf_source_frame = "odom",
                   const std::string& tf_target_frame = "base_link")
        : node_(node), base_body_name_(base_body_name), enabled_(enabled), 
          has_new_data_(false), use_tf_(false),
          odom_topic_(topic), tf_source_frame_(tf_source_frame),
          tf_target_frame_(tf_target_frame), child_frame_id_("base_link") {
        
        if (!enabled_) {
            return;
        }
        
        // Determine mode
        if (mode == "tf") {
            use_tf_ = true;
            // Create TF buffer and listener
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            
            RCLCPP_INFO(node_->get_logger(), 
                       "External odometry subscriber initialized (TF mode)");
            RCLCPP_INFO(node_->get_logger(), 
                       "  Listening to TF: %s -> %s", 
                       tf_source_frame_.c_str(), tf_target_frame_.c_str());
        } else {
            // Subscribe to odometry topic
            odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_, 10,
                std::bind(&OdomSubscriber::odomCallback, this, std::placeholders::_1));
            
            RCLCPP_INFO(node_->get_logger(), 
                       "External odometry subscriber initialized (Odom mode)");
            RCLCPP_INFO(node_->get_logger(), 
                       "  Topic: %s", odom_topic_.c_str());
        }
    }
    
    ~OdomSubscriber() = default;
    
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
                           "Found base body for external control: '%s' (id=%d)", 
                           base_names[i], base_body_id_);
                break;
            }
        }
        
        // If still not found, use first body after world (id=0)
        if (base_body_id_ < 0 && m->nbody > 1) {
            base_body_id_ = 1;  // First body after world
            RCLCPP_WARN(node_->get_logger(), 
                       "Using default base body for external control: '%s' (id=1)", 
                       mj_id2name(m, mjOBJ_BODY, base_body_id_));
        }
        
        if (base_body_id_ < 0) {
            RCLCPP_ERROR(node_->get_logger(), 
                        "Could not find any base body, external odometry control disabled");
            enabled_ = false;
            return;
        }
        
        RCLCPP_INFO(node_->get_logger(), 
                   "External odometry will control body: '%s' (id=%d)", 
                   mj_id2name(m, mjOBJ_BODY, base_body_id_), base_body_id_);
    }
    
    // Apply external odometry to robot base pose
    void applyToSimulation(mjModel* m, mjData* d) {
        if (!enabled_ || base_body_id_ < 0) return;
        
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // For TF mode, try to get latest transform
        if (use_tf_) {
            try {
                geometry_msgs::msg::TransformStamped transform = 
                    tf_buffer_->lookupTransform(tf_source_frame_, tf_target_frame_, 
                                               tf2::TimePointZero);
                
                // Update cached pose from transform
                cached_pos_[0] = transform.transform.translation.x;
                cached_pos_[1] = transform.transform.translation.y;
                cached_pos_[2] = transform.transform.translation.z;
                
                cached_quat_[0] = transform.transform.rotation.w;
                cached_quat_[1] = transform.transform.rotation.x;
                cached_quat_[2] = transform.transform.rotation.y;
                cached_quat_[3] = transform.transform.rotation.z;
                
                has_new_data_ = true;
            } catch (tf2::TransformException& ex) {
                // Transform not available yet, use cached data if available
                if (!has_new_data_) {
                    return;
                }
            }
        }
        
        if (!has_new_data_) return;
        
        // Method 1: Direct position/velocity control (kinematic mode)
        // This directly sets the base state, bypassing physics
        // For floating base robots, qpos starts with: [x, y, z, qw, qx, qy, qz, ...]
        
        // Position
        d->qpos[0] = cached_pos_[0];
        d->qpos[1] = cached_pos_[1];
        d->qpos[2] = cached_pos_[2];
        
        // Orientation (quaternion: MuJoCo uses w,x,y,z order)
        d->qpos[3] = cached_quat_[0];  // w
        d->qpos[4] = cached_quat_[1];  // x
        d->qpos[5] = cached_quat_[2];  // y
        d->qpos[6] = cached_quat_[3];  // z
        
        // Apply velocity if available
        if (has_velocity_) {
            d->qvel[0] = cached_vel_linear_[0];
            d->qvel[1] = cached_vel_linear_[1];
            d->qvel[2] = cached_vel_linear_[2];
            d->qvel[3] = cached_vel_angular_[0];
            d->qvel[4] = cached_vel_angular_[1];
            d->qvel[5] = cached_vel_angular_[2];
        } else {
            // If no velocity data, set base velocities to zero
            d->qvel[0] = 0.0;
            d->qvel[1] = 0.0;
            d->qvel[2] = 0.0;
            d->qvel[3] = 0.0;
            d->qvel[4] = 0.0;
            d->qvel[5] = 0.0;
        }
        
        // Call mj_forward to update dependent quantities without advancing time
        // This ensures xpos, xquat etc. are consistent with qpos
        mj_forward(m, d);
    }
    
    bool isEnabled() const { return enabled_; }
    bool hasNewData() const { return has_new_data_; }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Cache position
        cached_pos_[0] = msg->pose.pose.position.x;
        cached_pos_[1] = msg->pose.pose.position.y;
        cached_pos_[2] = msg->pose.pose.position.z;
        
        // Cache orientation (ROS uses xyzw, convert to wxyz)
        cached_quat_[0] = msg->pose.pose.orientation.w;
        cached_quat_[1] = msg->pose.pose.orientation.x;
        cached_quat_[2] = msg->pose.pose.orientation.y;
        cached_quat_[3] = msg->pose.pose.orientation.z;
        
        // Cache velocity
        cached_vel_linear_[0] = msg->twist.twist.linear.x;
        cached_vel_linear_[1] = msg->twist.twist.linear.y;
        cached_vel_linear_[2] = msg->twist.twist.linear.z;
        cached_vel_angular_[0] = msg->twist.twist.angular.x;
        cached_vel_angular_[1] = msg->twist.twist.angular.y;
        cached_vel_angular_[2] = msg->twist.twist.angular.z;
        
        has_velocity_ = true;
        has_new_data_ = true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // TF support
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string base_body_name_;
    std::string odom_topic_;
    std::string tf_source_frame_;
    std::string tf_target_frame_;
    std::string child_frame_id_;
    bool enabled_;
    bool use_tf_;
    bool has_new_data_;
    bool has_velocity_ = false;
    
    int base_body_id_ = -1;
    
    // Cached pose and velocity data
    std::mutex data_mutex_;
    double cached_pos_[3] = {0, 0, 0};
    double cached_quat_[4] = {1, 0, 0, 0};  // w, x, y, z
    double cached_vel_linear_[3] = {0, 0, 0};
    double cached_vel_angular_[3] = {0, 0, 0};
};
