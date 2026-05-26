#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>

#include "param.h"

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <sstream>
#include <algorithm>
#include <cmath>

class RaycasterPublisher {
public:
    enum class OutputFormat {
        POINTCLOUD,
        ARRAY,
        BOTH
    };
    
    RaycasterPublisher(rclcpp::Node::SharedPtr node, OutputFormat format = OutputFormat::POINTCLOUD, 
                       bool flatten_xyz = true, bool zero_mean = false) 
        : node_(node), publish_tf_(true), initialized_(false), 
          output_format_(format), flatten_xyz_(flatten_xyz), zero_mean_(zero_mean) {
        
        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    }
    
    ~RaycasterPublisher() = default;
    
    // Initialize raycaster sensors from the model
    void initialize(mjModel* m, mjData* d) {
        if (!m || !d) return;

        raycaster_sensors_.clear();
        camera_sensors_.clear();
        
        // Run a few simulation steps to ensure plugins are fully initialized
        for (int i = 0; i < 3; i++) {
            mj_step(m, d);
        }
        
        for (int i = 0; i < m->nsensor; i++) {
            const char* sensor_name = mj_id2name(m, mjOBJ_SENSOR, i);
            if (!sensor_name) continue;
            
            int plugin_id = m->sensor_plugin[i];
            if (plugin_id < 0) continue;
            
            const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[plugin_id]);
            if (!plugin || !plugin->name) continue;
            
            std::string plugin_name(plugin->name);
            
            // Check if it's a raycaster plugin
            if (plugin_name.find("mujoco.sensor.ray_caster") != std::string::npos) {
                RayCasterSensor sensor;
                sensor.name = sensor_name;
                sensor.sensor_id = i;
                sensor.data_adr = m->sensor_adr[i];
                sensor.plugin_id = plugin_id;
                sensor.state_adr = m->plugin_stateadr[plugin_id];
                
                // Get the object name that this sensor is attached to
                // This is used to find corresponding pos/quat sensors
                int objtype = m->sensor_type[i];
                int objid = m->sensor_objid[i];
                const char* obj_name = nullptr;
                if (objtype == mjOBJ_CAMERA && objid >= 0) {
                    obj_name = mj_id2name(m, mjOBJ_CAMERA, objid);
                } else if (objtype == mjOBJ_SITE && objid >= 0) {
                    obj_name = mj_id2name(m, mjOBJ_SITE, objid);
                }
                if (obj_name) {
                    sensor.attached_obj_name = std::string(obj_name);
                }
                
                // Determine frame_id based on data type
                // Will be updated after reading sensor_data_types config
                sensor.frame_id = "world";
                
                // Read sensor_data_types configuration from plugin
                const char* config_str = mj_getPluginConfig(m, plugin_id, "sensor_data_types");
                std::vector<std::string> sensor_data_types;
                if (config_str) {
                    sensor_data_types = parse_sensor_data_types(std::string(config_str));
                    RCLCPP_INFO(node_->get_logger(), 
                        "  sensor_data_types config: %s", config_str);
                }
                
                // Read dis_range configuration from plugin (min_distance, max_distance)
                const char* dis_range_str = mj_getPluginConfig(m, plugin_id, "dis_range");
                float min_distance = 0.1f;
                float max_distance = 10.0f;
                if (dis_range_str) {
                    std::stringstream ss(dis_range_str);
                    ss >> min_distance >> max_distance;
                    RCLCPP_INFO(node_->get_logger(), 
                        "  dis_range config: [%.2f, %.2f]", min_distance, max_distance);
                }
                
                // Get raycaster info from plugin state
                if (d && sensor.state_adr >= 0 && m->plugin_statenum[plugin_id] >= 2) {
                    sensor.h_ray_num = static_cast<int>(d->plugin_state[sensor.state_adr + 0]);
                    sensor.v_ray_num = static_cast<int>(d->plugin_state[sensor.state_adr + 1]);
                    
                    // Get data configuration from plugin state
                    std::vector<std::pair<int, int>> data_configs;
                    for (int j = sensor.state_adr + 2; 
                         j < sensor.state_adr + m->plugin_statenum[plugin_id]; 
                         j += 2) {
                        int offset = static_cast<int>(d->plugin_state[j]);
                        int size = static_cast<int>(d->plugin_state[j + 1]);
                        data_configs.push_back({offset, size});
                    }
                    
                    // Match data configs with sensor_data_types configuration
                    int nray = sensor.h_ray_num * sensor.v_ray_num;
                    bool found_data = false;
                    
                    // Look for "data" type first (preferred for publishing)
                    std::string selected_data_type;
                    for (size_t j = 0; j < sensor_data_types.size() && j < data_configs.size(); j++) {
                        if (sensor_data_types[j].find("data") != std::string::npos) {
                            sensor.pos_w_data_offset = data_configs[j].first;
                            sensor.pos_w_data_size = data_configs[j].second;
                            sensor.data_type = get_data_type_from_string(sensor_data_types[j]);
                            selected_data_type = sensor_data_types[j];
                            found_data = true;
                            RCLCPP_INFO(node_->get_logger(), 
                                "  Using '%s' type (config index %zu, offset=%d, size=%d)",
                                sensor_data_types[j].c_str(), j, 
                                sensor.pos_w_data_offset, sensor.pos_w_data_size);
                            break;
                        }
                    }
                    
                    // If no "data" type, look for pos_w or pos_b
                    if (!found_data) {
                        for (size_t j = 0; j < sensor_data_types.size() && j < data_configs.size(); j++) {
                            if (sensor_data_types[j].find("pos_w") != std::string::npos || 
                                sensor_data_types[j].find("pos_b") != std::string::npos) {
                                sensor.pos_w_data_offset = data_configs[j].first;
                                sensor.pos_w_data_size = data_configs[j].second;
                                sensor.data_type = get_data_type_from_string(sensor_data_types[j]);
                                selected_data_type = sensor_data_types[j];
                                found_data = true;
                                RCLCPP_INFO(node_->get_logger(), 
                                    "  Using '%s' type (config index %zu, offset=%d, size=%d)",
                                    sensor_data_types[j].c_str(), j,
                                    sensor.pos_w_data_offset, sensor.pos_w_data_size);
                                break;
                            }
                        }
                    }
                    
                    // Update frame_id based on data type
                    // pos_b: data in body/sensor frame, pos_w: data in world frame
                    if (selected_data_type.find("pos_b") != std::string::npos) {
                        sensor.frame_id = sensor_name;  // Use sensor name as frame
                    } else {
                        sensor.frame_id = "world";  // pos_w or other data types use world frame
                    }
                    
                    // If still not found, use first available type
                    if (!found_data && !sensor_data_types.empty() && !data_configs.empty()) {
                        sensor.pos_w_data_offset = data_configs[0].first;
                        sensor.pos_w_data_size = data_configs[0].second;
                        sensor.data_type = get_data_type_from_string(sensor_data_types[0]);
                        selected_data_type = sensor_data_types[0];
                        found_data = true;
                        RCLCPP_INFO(node_->get_logger(), 
                            "  Using '%s' type (first available, offset=%d, size=%d)",
                            sensor_data_types[0].c_str(),
                            sensor.pos_w_data_offset, sensor.pos_w_data_size);
                        
                        // Update frame_id based on data type
                        if (selected_data_type.find("pos_b") != std::string::npos) {
                            sensor.frame_id = sensor_name;
                        }
                    }
                    
                    if (!found_data) {
                        sensor.pos_w_data_offset = 0;
                        sensor.pos_w_data_size = nray * 3;
                        sensor.data_type = SensorDataType::POSITION_3D;
                        RCLCPP_WARN(node_->get_logger(), 
                            "No valid data config found for %s, using defaults", sensor_name);
                    }
                }
                
                // Create publisher using sensor name as topic
                std::string topic_name = "/" + std::string(sensor_name);
                
                // Get sensor-specific configuration or use defaults
                std::string output_format_str = param::config.raycaster_output_format;
                bool flatten_xyz = param::config.raycaster_flatten_xyz;
                bool zero_mean = param::config.raycaster_zero_mean;
                float offset = 0.0f;
                std::string replace_nan = "";
                std::string distance_type = "";
                
                // Check if sensor has specific configuration
                auto it = param::config.raycaster_sensors.find(sensor_name);
                if (it != param::config.raycaster_sensors.end()) {
                    if (!it->second.output_format.empty()) {
                        output_format_str = it->second.output_format;
                    }
                    flatten_xyz = it->second.flatten_xyz;
                    zero_mean = it->second.zero_mean;
                    offset = it->second.offset;
                    replace_nan = it->second.replace_nan;
                    distance_type = it->second.distance_type;
                    // Note: max_distance from param config is ignored, we use MuJoCo's dis_range instead
                }
                
                // Determine output format for this sensor
                OutputFormat sensor_format = OutputFormat::POINTCLOUD;
                if (output_format_str == "array") {
                    sensor_format = OutputFormat::ARRAY;
                } else if (output_format_str == "both") {
                    sensor_format = OutputFormat::BOTH;
                }
                
                sensor.output_format = sensor_format;
                sensor.flatten_xyz = flatten_xyz;
                sensor.zero_mean = zero_mean;
                sensor.offset = offset;
                sensor.replace_nan = replace_nan;
                sensor.max_distance = max_distance;  // From MuJoCo dis_range config
                sensor.distance_type = distance_type;
                
                std::string log_topic_name = topic_name;
                if (sensor_format == OutputFormat::POINTCLOUD || sensor_format == OutputFormat::BOTH) {
                    sensor.pc_publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                        topic_name, 10);
                    
                    // Pre-allocate message template to avoid repeated allocation
                    int num_points = sensor.h_ray_num * sensor.v_ray_num;
                    sensor.pc_msg_template.header.frame_id = sensor.frame_id;
                    sensor.pc_msg_template.height = 1;
                    sensor.pc_msg_template.point_step = 12;
                    sensor.pc_msg_template.is_dense = false;
                    
                    sensor.pc_msg_template.fields.resize(3);
                    sensor.pc_msg_template.fields[0].name = "x";
                    sensor.pc_msg_template.fields[0].offset = 0;
                    sensor.pc_msg_template.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    sensor.pc_msg_template.fields[0].count = 1;
                    
                    sensor.pc_msg_template.fields[1].name = "y";
                    sensor.pc_msg_template.fields[1].offset = 4;
                    sensor.pc_msg_template.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    sensor.pc_msg_template.fields[1].count = 1;
                    
                    sensor.pc_msg_template.fields[2].name = "z";
                    sensor.pc_msg_template.fields[2].offset = 8;
                    sensor.pc_msg_template.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    sensor.pc_msg_template.fields[2].count = 1;
                    
                    sensor.pc_msg_template.data.resize(num_points * 12);
                }

                if (sensor_format == OutputFormat::ARRAY || sensor_format == OutputFormat::BOTH) {
                    std::string array_topic_name = topic_name + "_array";
                    sensor.array_publisher = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
                        array_topic_name, 10);
                    
                    int num_points = sensor.h_ray_num * sensor.v_ray_num;
                    if (sensor.flatten_xyz) {
                        sensor.array_msg_template.data.reserve(num_points * 3);
                    } else {
                        sensor.array_msg_template.data.reserve(num_points);
                    }

                    if (sensor_format == OutputFormat::BOTH) {
                        log_topic_name += " + " + array_topic_name;
                    } else {
                        log_topic_name = array_topic_name;
                    }
                }
                
                RCLCPP_INFO(node_->get_logger(), "RayCaster sensor: %s (topic: %s, frame: %s, %dx%d rays)",
                    sensor_name, log_topic_name.c_str(), sensor.frame_id.c_str(),
                    sensor.h_ray_num, sensor.v_ray_num);
                
                raycaster_sensors_.push_back(std::move(sensor));
            }
        }
        
        // Find camera sensors for TF
        if (publish_tf_) {
            find_camera_sensors(m);
        }
        
        RCLCPP_INFO(node_->get_logger(), "Raycaster publisher initialized with %zu sensor(s)", 
                    raycaster_sensors_.size());
        
        initialized_ = true;
    }
    
    // Publish raycaster data and TF
    void publish(mjModel* m, mjData* d) {
        if (!m || !d || !initialized_) return;
        
        publish_raycaster_data(d);
        
        if (publish_tf_) {
            publish_transforms(d);
        }
    }

private:
    enum class SensorDataType {
        POSITION_3D,  // pos_w or pos_b: 3 values per ray (x, y, z)
        DISTANCE_1D   // data, image, or normal: 1 value per ray
    };
    
    // Helper function to parse sensor_data_types configuration
    static std::vector<std::string> parse_sensor_data_types(const std::string& config_str) {
        std::vector<std::string> result;
        std::stringstream ss(config_str);
        std::string item;
        while (ss >> item) {
            if (!item.empty()) {
                result.push_back(item);
            }
        }
        return result;
    }
    
    // Helper function to determine data type from string
    static SensorDataType get_data_type_from_string(const std::string& type_str) {
        if (type_str.find("pos_w") != std::string::npos || 
            type_str.find("pos_b") != std::string::npos) {
            return SensorDataType::POSITION_3D;
        } else if (type_str.find("data") != std::string::npos || 
                   type_str.find("image") != std::string::npos || 
                   type_str.find("normal") != std::string::npos) {
            return SensorDataType::DISTANCE_1D;
        }
        return SensorDataType::DISTANCE_1D; // default
    }
    
    struct RayCasterSensor {
        std::string name;
        int sensor_id;
        int data_adr;
        int plugin_id;
        int state_adr;
        int h_ray_num;
        int v_ray_num;
        int pos_w_data_offset;
        int pos_w_data_size;
        std::string frame_id;
        std::string attached_obj_name;  // Name of camera/site object this sensor is attached to
        SensorDataType data_type;  // Type of sensor data
        
        // Per-sensor configuration
        OutputFormat output_format;
        bool flatten_xyz;
        bool zero_mean;
        float offset;              // Offset to apply to distances
        std::string replace_nan;  // "zero", "max", or ""
        float max_distance;        // Maximum distance from sensor config
        std::string distance_type;
        
        // Publishers for different formats
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr array_publisher;
        
        // Pre-allocated buffers
        sensor_msgs::msg::PointCloud2 pc_msg_template;
        std_msgs::msg::Float32MultiArray array_msg_template;
    };
    
    struct CameraSensor {
        std::string name;
        int quat_sensor_id;
        int pos_sensor_id;
        int quat_adr;
        int pos_adr;
        std::string frame_id;
    };
    
    // Member variables
    rclcpp::Node::SharedPtr node_;
    bool publish_tf_;
    bool initialized_;
    OutputFormat output_format_;
    bool flatten_xyz_;
    bool zero_mean_;
    
    std::vector<RayCasterSensor> raycaster_sensors_;
    std::map<std::string, CameraSensor> camera_sensors_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Private methods
    void find_camera_sensors(mjModel* m) {
        // First pass: find sensors with _pos and _quat suffixes
        for (int i = 0; i < m->nsensor; i++) {
            const char* sensor_name = mj_id2name(m, mjOBJ_SENSOR, i);
            if (!sensor_name) continue;
            
            std::string name_str(sensor_name);
            
            // Check for position sensor
            if (name_str.find("_pos") != std::string::npos) {
                std::string base_name = name_str.substr(0, name_str.find("_pos"));
                
                if (camera_sensors_.find(base_name) == camera_sensors_.end()) {
                    camera_sensors_[base_name] = CameraSensor();
                    camera_sensors_[base_name].name = base_name;
                    camera_sensors_[base_name].frame_id = base_name;
                    camera_sensors_[base_name].quat_sensor_id = -1;
                    camera_sensors_[base_name].pos_sensor_id = -1;
                    camera_sensors_[base_name].quat_adr = -1;
                    camera_sensors_[base_name].pos_adr = -1;
                }
                
                camera_sensors_[base_name].pos_sensor_id = i;
                camera_sensors_[base_name].pos_adr = m->sensor_adr[i];
            }
            
            // Check for quaternion sensor
            if (name_str.find("_quat") != std::string::npos) {
                std::string base_name = name_str.substr(0, name_str.find("_quat"));
                
                if (camera_sensors_.find(base_name) == camera_sensors_.end()) {
                    camera_sensors_[base_name] = CameraSensor();
                    camera_sensors_[base_name].name = base_name;
                    camera_sensors_[base_name].frame_id = base_name;
                    camera_sensors_[base_name].quat_sensor_id = -1;
                    camera_sensors_[base_name].pos_sensor_id = -1;
                    camera_sensors_[base_name].quat_adr = -1;
                    camera_sensors_[base_name].pos_adr = -1;
                }
                
                camera_sensors_[base_name].quat_sensor_id = i;
                camera_sensors_[base_name].quat_adr = m->sensor_adr[i];
            }
        }
        
        // Second pass: add raycaster sensors that need TF (those using pos_b frame)
        for (const auto& raycaster : raycaster_sensors_) {
            // If raycaster uses its own frame (pos_b), add it to camera_sensors for TF publishing
            if (raycaster.frame_id != "world" && 
                camera_sensors_.find(raycaster.name) == camera_sensors_.end()) {
                
                // Try multiple naming patterns to find corresponding pos/quat sensors
                std::vector<std::string> name_candidates;
                
                // Pattern 1: sensor_name + "_pos"/"_quat"
                name_candidates.push_back(raycaster.name);
                
                // Pattern 2: attached_obj_name + "_pos"/"_quat" (e.g., "ray_caster_camera")
                if (!raycaster.attached_obj_name.empty()) {
                    name_candidates.push_back(raycaster.attached_obj_name);
                }
                
                int pos_id = -1;
                int quat_id = -1;
                std::string matched_name;
                
                for (const auto& base_name : name_candidates) {
                    std::string pos_sensor_name = base_name + "_pos";
                    std::string quat_sensor_name = base_name + "_quat";
                    
                    pos_id = mj_name2id(m, mjOBJ_SENSOR, pos_sensor_name.c_str());
                    quat_id = mj_name2id(m, mjOBJ_SENSOR, quat_sensor_name.c_str());
                    
                    if (pos_id >= 0 && quat_id >= 0) {
                        matched_name = base_name;
                        break;
                    }
                }
                
                if (pos_id >= 0 && quat_id >= 0) {
                    CameraSensor cam_sensor;
                    cam_sensor.name = raycaster.name;
                    cam_sensor.frame_id = raycaster.frame_id;
                    cam_sensor.pos_sensor_id = pos_id;
                    cam_sensor.quat_sensor_id = quat_id;
                    cam_sensor.pos_adr = m->sensor_adr[pos_id];
                    cam_sensor.quat_adr = m->sensor_adr[quat_id];
                    
                    camera_sensors_[raycaster.name] = cam_sensor;
                    
                    RCLCPP_INFO(node_->get_logger(), 
                        "  Added TF for raycaster '%s' using sensors '%s_pos/%s_quat'", 
                        raycaster.name.c_str(), matched_name.c_str(), matched_name.c_str());
                } else {
                    RCLCPP_WARN(node_->get_logger(), 
                        "  Could not find pos/quat sensors for raycaster '%s' (tried: %s, %s)",
                        raycaster.name.c_str(), 
                        raycaster.name.c_str(),
                        raycaster.attached_obj_name.c_str());
                }
            }
        }
        
        RCLCPP_INFO(node_->get_logger(), "Found %zu camera/raycaster sensor(s) for TF", 
                    camera_sensors_.size());
    }
    
    void publish_raycaster_data(mjData* d) {
        for (auto& sensor : raycaster_sensors_) {
            if (sensor.output_format == OutputFormat::POINTCLOUD || sensor.output_format == OutputFormat::BOTH) {
                publish_sensor_as_pointcloud(sensor, d);
            }
            if (sensor.output_format == OutputFormat::ARRAY || sensor.output_format == OutputFormat::BOTH) {
                publish_sensor_as_array(sensor, d);
            }
        }
    }
    
    void publish_sensor_as_pointcloud(RayCasterSensor& sensor, mjData* d) {
        auto& msg = sensor.pc_msg_template;
        msg.header.stamp = node_->now();
            
        int num_points = sensor.h_ray_num * sensor.v_ray_num;
        
        // Get data pointer
        mjtNum* data_ptr = nullptr;
        if (sensor.pos_w_data_size > 0 && sensor.pos_w_data_offset >= 0) {
            data_ptr = d->sensordata + sensor.data_adr + sensor.pos_w_data_offset;
        } else {
            data_ptr = d->sensordata + sensor.data_adr;
        }
        
        // Fast copy with type conversion and offset application
        float* msg_data_ptr = reinterpret_cast<float*>(msg.data.data());
        
        if (sensor.data_type == SensorDataType::POSITION_3D) {
            // Data is 3D positions (x, y, z)
            for (int i = 0; i < num_points; i++) {
                msg_data_ptr[i * 3 + 0] = static_cast<float>(data_ptr[i * 3 + 0]);
                msg_data_ptr[i * 3 + 1] = static_cast<float>(data_ptr[i * 3 + 1]);
                msg_data_ptr[i * 3 + 2] = static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset;
            }
        } else {
            // Data is 1D distances - we can't publish as pointcloud meaningfully
            // For now, just publish as (0, 0, distance)
            for (int i = 0; i < num_points; i++) {
                msg_data_ptr[i * 3 + 0] = 0.0f;
                msg_data_ptr[i * 3 + 1] = 0.0f;
                msg_data_ptr[i * 3 + 2] = static_cast<float>(data_ptr[i]) + sensor.offset;
            }
        }

        msg.width = num_points;
        msg.row_step = msg.point_step * num_points;

        sensor.pc_publisher->publish(msg);
    }

    void publish_sensor_as_array(RayCasterSensor& sensor, mjData* d) {
        auto& msg = sensor.array_msg_template;
        msg.data.clear();

        int num_points = sensor.h_ray_num * sensor.v_ray_num;

        // Get data pointer
        mjtNum* data_ptr = nullptr;
        if (sensor.pos_w_data_size > 0 && sensor.pos_w_data_offset >= 0) {
            data_ptr = d->sensordata + sensor.data_adr + sensor.pos_w_data_offset;
        } else {
            data_ptr = d->sensordata + sensor.data_adr;
        }

        // Helper function to replace non-finite values
        auto replace_nonfinite = [&](float value) -> float {
            if (!std::isfinite(value)) {
                if (sensor.replace_nan == "zero") {
                    return 0.0f;
                } else if (sensor.replace_nan == "max") {
                    return sensor.max_distance;
                }
            }
            return value;
        };

        if (sensor.data_type == SensorDataType::POSITION_3D) {
            // Data is 3D positions (x, y, z)
            if (sensor.flatten_xyz) {
                // Flatten all x, y, z values: [x0, y0, z0, x1, y1, z1, ...]
                msg.data.reserve(num_points * 3);

                if (sensor.zero_mean) {
                    // First pass: compute mean of z values (after offset)
                    float z_sum = 0.0f;
                    int z_count = 0;
                    for (int i = 0; i < num_points; i++) {
                        float z = replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset);
                        if (std::isfinite(z)) {
                            z_sum += z;
                            z_count++;
                        }
                    }
                    float z_mean = (z_count > 0) ? (z_sum / z_count) : 0.0f;

                    // Second pass: subtract mean from z
                    for (int i = 0; i < num_points; i++) {
                        msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 0])));
                        msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 1])));
                        msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset) - z_mean);
                    }
                } else {
                    // No mean subtraction, but apply offset
                    for (int i = 0; i < num_points; i++) {
                        msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 0])));
                        msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 1])));
                        msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset));
                    }
                }
            } else {
                // Single value per ray
                msg.data.reserve(num_points);

                if (sensor.distance_type == "eu") {
                    // Calculate euclidean distance: sqrt(x^2 + y^2 + z^2), always >= 0
                    if (sensor.zero_mean) {
                        // Compute mean (after offset)
                        float dist_sum = 0.0f;
                        int dist_count = 0;
                        for (int i = 0; i < num_points; i++) {
                            float x = static_cast<float>(data_ptr[i * 3 + 0]);
                            float y = static_cast<float>(data_ptr[i * 3 + 1]);
                            float z = static_cast<float>(data_ptr[i * 3 + 2]);
                            float dist = replace_nonfinite(std::sqrt(x*x + y*y + z*z) + sensor.offset);
                            if (std::isfinite(dist)) {
                                dist_sum += dist;
                                dist_count++;
                            }
                        }
                        float dist_mean = (dist_count > 0) ? (dist_sum / dist_count) : 0.0f;

                        // Subtract mean
                        for (int i = 0; i < num_points; i++) {
                            float x = static_cast<float>(data_ptr[i * 3 + 0]);
                            float y = static_cast<float>(data_ptr[i * 3 + 1]);
                            float z = static_cast<float>(data_ptr[i * 3 + 2]);
                            msg.data.push_back(replace_nonfinite(std::sqrt(x*x + y*y + z*z) + sensor.offset) - dist_mean);
                        }
                    } else {
                        // No mean subtraction, but apply offset
                        for (int i = 0; i < num_points; i++) {
                            float x = static_cast<float>(data_ptr[i * 3 + 0]);
                            float y = static_cast<float>(data_ptr[i * 3 + 1]);
                            float z = static_cast<float>(data_ptr[i * 3 + 2]);
                            msg.data.push_back(replace_nonfinite(std::sqrt(x*x + y*y + z*z) + sensor.offset));
                        }
                    }
                } else if (sensor.distance_type == "abs"){
                    // Use z coordinate absolute value (always >= 0)
                    if (sensor.zero_mean) {
                        // Compute mean (after offset and abs)
                        float z_sum = 0.0f;
                        int z_count = 0;
                        for (int i = 0; i < num_points; i++) {
                            float z = replace_nonfinite(std::abs(static_cast<float>(data_ptr[i * 3 + 2])) + sensor.offset);
                            if (std::isfinite(z)) {
                                z_sum += z;
                                z_count++;
                            }
                        }
                        float z_mean = (z_count > 0) ? (z_sum / z_count) : 0.0f;

                        // Subtract mean
                        for (int i = 0; i < num_points; i++) {
                            msg.data.push_back(replace_nonfinite(std::abs(static_cast<float>(data_ptr[i * 3 + 2])) + sensor.offset) - z_mean);
                        }
                    } else {
                        // No mean subtraction, but apply offset and abs
                        for (int i = 0; i < num_points; i++) {
                            msg.data.push_back(replace_nonfinite(std::abs(static_cast<float>(data_ptr[i * 3 + 2])) + sensor.offset));
                        }
                    }
                } else {
                    // Default: use z coordinate directly
                    if (sensor.zero_mean) {
                        // Compute mean (after offset)
                        float z_sum = 0.0f;
                        int z_count = 0;
                        for (int i = 0; i < num_points; i++) {
                            float z = replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset);
                            if (std::isfinite(z)) {
                                z_sum += z;
                                z_count++;
                            }
                        }
                        float z_mean = (z_count > 0) ? (z_sum / z_count) : 0.0f;

                        // Subtract mean
                        for (int i = 0; i < num_points; i++) {
                            msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset) - z_mean);
                        }
                    } else {
                        // No mean subtraction, but apply offset
                        for (int i = 0; i < num_points; i++) {
                            msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset));
                        }
                    }
                }
            }
        } else {
            // Data is 1D distances
            msg.data.reserve(num_points);

            if (sensor.zero_mean) {
                // Compute mean (after offset)
                float sum = 0.0f;
                int count = 0;
                for (int i = 0; i < num_points; i++) {
                    float val = replace_nonfinite(static_cast<float>(data_ptr[i]) + sensor.offset);
                    if (std::isfinite(val)) {
                        sum += val;
                        count++;
                    }
                }
                float mean = (count > 0) ? (sum / count) : 0.0f;

                // Subtract mean
                for (int i = 0; i < num_points; i++) {
                    msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i]) + sensor.offset) - mean);
                }
            } else {
                // No mean subtraction, but apply offset
                for (int i = 0; i < num_points; i++) {
                    msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i]) + sensor.offset));
                }
            }
        }
        
        sensor.array_publisher->publish(msg);
    }
    
    void publish_transforms(mjData* d) {
        auto now = node_->now();
        
        for (const auto& [name, camera] : camera_sensors_) {
            if (camera.pos_adr < 0 || camera.quat_adr < 0) continue;
            
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "world";
            t.child_frame_id = camera.frame_id;
            
            // Position
            t.transform.translation.x = d->sensordata[camera.pos_adr + 0];
            t.transform.translation.y = d->sensordata[camera.pos_adr + 1];
            t.transform.translation.z = d->sensordata[camera.pos_adr + 2];
            
            // Quaternion
            t.transform.rotation.w = d->sensordata[camera.quat_adr + 0];
            t.transform.rotation.x = d->sensordata[camera.quat_adr + 1];
            t.transform.rotation.y = d->sensordata[camera.quat_adr + 2];
            t.transform.rotation.z = d->sensordata[camera.quat_adr + 3];
            
            tf_broadcaster_->sendTransform(t);
        }
    }
};
