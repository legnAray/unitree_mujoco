#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <cmath>

class GridMapPublisher {
public:
    GridMapPublisher(rclcpp::Node::SharedPtr node, 
                     const std::string& input_topic = "/height_scan",
                     const std::string& output_topic = "/elevation_map")
        : node_(node), enabled_(true) {
        
        // Declare parameters
        node_->declare_parameter("gridmap.enabled", true);
        node_->declare_parameter("gridmap.map_frame", "world");
        node_->declare_parameter("gridmap.resolution", 0.05);
        node_->declare_parameter("gridmap.min_x", -2.0);
        node_->declare_parameter("gridmap.max_x", 2.0);
        node_->declare_parameter("gridmap.min_y", -2.0);
        node_->declare_parameter("gridmap.max_y", 2.0);
        node_->declare_parameter("gridmap.min_points_per_cell", 1);
        node_->declare_parameter("gridmap.default_uncertainty", 0.01);
        node_->declare_parameter("gridmap.max_uncertainty", 0.5);
        node_->declare_parameter("gridmap.height_offset", 0.0);
        
        // Get parameters
        enabled_ = node_->get_parameter("gridmap.enabled").as_bool();
        map_frame_ = node_->get_parameter("gridmap.map_frame").as_string();
        resolution_ = node_->get_parameter("gridmap.resolution").as_double();
        min_x_ = node_->get_parameter("gridmap.min_x").as_double();
        max_x_ = node_->get_parameter("gridmap.max_x").as_double();
        min_y_ = node_->get_parameter("gridmap.min_y").as_double();
        max_y_ = node_->get_parameter("gridmap.max_y").as_double();
        min_points_per_cell_ = node_->get_parameter("gridmap.min_points_per_cell").as_int();
        default_uncertainty_ = node_->get_parameter("gridmap.default_uncertainty").as_double();
        max_uncertainty_ = node_->get_parameter("gridmap.max_uncertainty").as_double();
        height_offset_ = node_->get_parameter("gridmap.height_offset").as_double();
        
        if (!enabled_) {
            RCLCPP_INFO(node_->get_logger(), "GridMap publisher is disabled");
            return;
        }
        
        // Create subscription and publisher
        pc_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&GridMapPublisher::pointCloudCallback, this, std::placeholders::_1));
        
        gridmap_pub_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(output_topic, 10);
        
        RCLCPP_INFO(node_->get_logger(), "GridMap publisher initialized");
        RCLCPP_INFO(node_->get_logger(), "  Input: %s", input_topic.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Output: %s", output_topic.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Frame: %s", map_frame_.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Resolution: %.3fm", resolution_);
        RCLCPP_INFO(node_->get_logger(), "  Map bounds: x[%.2f, %.2f] y[%.2f, %.2f]", 
                    min_x_, max_x_, min_y_, max_y_);
        RCLCPP_INFO(node_->get_logger(), "  Height offset: %.3fm", height_offset_);
    }
    
    ~GridMapPublisher() = default;
    
    bool isEnabled() const { return enabled_; }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!enabled_) return;
        
        // Convert to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->points.empty()) {
            return;
        }
        
        // Create GridMap
        grid_map::GridMap map({"elevation", "uncertainty"});
        map.setFrameId(map_frame_);
        map.setTimestamp(rclcpp::Time(msg->header.stamp).nanoseconds());
        
        // Set map geometry
        grid_map::Length length(max_x_ - min_x_, max_y_ - min_y_);
        grid_map::Position position((max_x_ + min_x_) / 2.0, (max_y_ + min_y_) / 2.0);
        map.setGeometry(length, resolution_, position);
        
        // Initialize layers
        map["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());
        map["uncertainty"].setConstant(static_cast<float>(default_uncertainty_));
        
        // Collect heights for each grid cell
        std::map<grid_map::Index, std::vector<float>, IndexCompare> cell_heights;
        
        for (const auto& point : cloud->points) {
            // Skip invalid points
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            
            grid_map::Position pos(point.x, point.y);
            
            if (!map.isInside(pos)) continue;
            
            grid_map::Index index;
            if (!map.getIndex(pos, index)) continue;
            
            cell_heights[index].push_back(point.z);
        }
        
        // Calculate elevation and uncertainty for each cell
        for (const auto& [index, heights] : cell_heights) {
            if (heights.size() < static_cast<size_t>(min_points_per_cell_)) continue;
            
            // Calculate mean elevation
            float sum = 0.0f;
            for (float h : heights) sum += h;
            float mean = sum / heights.size();
            
            // Apply height offset
            mean += static_cast<float>(height_offset_);
            
            // Calculate standard deviation as uncertainty
            float variance = 0.0f;
            for (float h : heights) {
                float diff = h - mean;
                variance += diff * diff;
            }
            float stddev = (heights.size() > 1) ? std::sqrt(variance / (heights.size() - 1)) : 0.0f;
            
            map.at("elevation", index) = mean;
            // Cap uncertainty at max_uncertainty
            map.at("uncertainty", index) = std::min(stddev, static_cast<float>(max_uncertainty_));
        }
        
        // Publish GridMap
        auto gridmap_msg = grid_map::GridMapRosConverter::toMessage(map);
        gridmap_pub_->publish(std::move(gridmap_msg));
    }
    
    // Custom comparator for grid_map::Index (Eigen::Array2i)
    struct IndexCompare {
        bool operator()(const grid_map::Index& a, const grid_map::Index& b) const {
            if (a(0) != b(0)) return a(0) < b(0);
            return a(1) < b(1);
        }
    };
    
    rclcpp::Node::SharedPtr node_;
    bool enabled_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
    
    std::string map_frame_;
    double resolution_;
    double min_x_, max_x_, min_y_, max_y_;
    int min_points_per_cell_;
    double default_uncertainty_;
    double max_uncertainty_;
    double height_offset_;
};
