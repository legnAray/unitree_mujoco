#pragma once

#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>
#include "simulate.h"
#include "lodepng/lodepng.h"
#include <string>
#include <vector>
#include <memory>
#include <cstring>
#include <iostream>
#include <sstream>

class DepthImageVisualizer {
public:
    DepthImageVisualizer(int display_scale = 2) : display_scale_(display_scale) {
        if (display_scale_ < 1) display_scale_ = 1;
        if (display_scale_ > 4) display_scale_ = 4;
    }
    ~DepthImageVisualizer() = default;
    
private:
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
    
public:
    
    void initialize(mjModel* m, mjData* d, mujoco::Simulate* sim) {
        if (!m || !d || !sim) return;
        
        sim_ = sim;
        depth_cameras_.clear();
        
        // Run a few simulation steps to ensure plugins are fully initialized
        for (int i = 0; i < 3; i++) {
            mj_step(m, d);
        }
        
        std::cout << "Searching for depth camera sensors..." << std::endl;
        
        // Find depth camera sensors with image data
        for (int i = 0; i < m->nsensor; i++) {
            const char* sensor_name = mj_id2name(m, mjOBJ_SENSOR, i);
            if (!sensor_name) continue;
            
            int plugin_id = m->sensor_plugin[i];
            if (plugin_id < 0) continue;
            
            const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[plugin_id]);
            if (!plugin || !plugin->name) continue;
            
            std::string plugin_name(plugin->name);
            
            // Check if it's a ray_caster_camera plugin
            if (plugin_name.find("mujoco.sensor.ray_caster_camera") != std::string::npos ||
                plugin_name.find("mujoco.sensor.ray_caster") != std::string::npos) {
                
                std::cout << "Found raycaster sensor: " << sensor_name 
                          << " (plugin: " << plugin_name << ")" << std::endl;
                
                DepthCameraSensor sensor;
                sensor.name = sensor_name;
                sensor.sensor_id = i;
                sensor.data_adr = m->sensor_adr[i];
                sensor.plugin_id = plugin_id;
                sensor.state_adr = m->plugin_stateadr[plugin_id];
                
                // Read sensor_data_types configuration from plugin
                const char* config_str = mj_getPluginConfig(m, plugin_id, "sensor_data_types");
                std::vector<std::string> sensor_data_types;
                if (config_str) {
                    sensor_data_types = parse_sensor_data_types(std::string(config_str));
                    std::cout << "  sensor_data_types config: " << config_str << std::endl;
                }
                
                // Read dis_range configuration from plugin
                const char* dis_range_str = mj_getPluginConfig(m, plugin_id, "dis_range");
                if (dis_range_str) {
                    std::cout << "  dis_range config: " << dis_range_str << std::endl;
                }
                
                // Get image dimensions from plugin state
                if (d && sensor.state_adr >= 0 && m->plugin_statenum[plugin_id] >= 2) {
                    sensor.width = static_cast<int>(d->plugin_state[sensor.state_adr + 0]);
                    sensor.height = static_cast<int>(d->plugin_state[sensor.state_adr + 1]);
                    
                    std::cout << "  Dimensions: " << sensor.width << "x" << sensor.height << std::endl;
                    std::cout << "  State num: " << m->plugin_statenum[plugin_id] << std::endl;
                    
                    // Get data configuration from plugin state
                    // State layout: [width, height, data1_offset, data1_size, data2_offset, data2_size, ...]
                    std::vector<std::pair<int, int>> data_configs;
                    for (int j = sensor.state_adr + 2; 
                         j < sensor.state_adr + m->plugin_statenum[plugin_id]; 
                         j += 2) {
                        int offset = static_cast<int>(d->plugin_state[j]);
                        int size = static_cast<int>(d->plugin_state[j + 1]);
                        data_configs.push_back({offset, size});
                    }
                    
                    std::cout << "  Number of data configs: " << data_configs.size() << std::endl;
                    
                    // Match data configs with sensor_data_types configuration
                    // Look for "image" type first (preferred for visualization)
                    bool found_image = false;
                    for (size_t j = 0; j < sensor_data_types.size() && j < data_configs.size(); j++) {
                        std::cout << "  Data config " << j << ": type='" << sensor_data_types[j] 
                                  << "', offset=" << data_configs[j].first 
                                  << ", size=" << data_configs[j].second << std::endl;
                        
                        if (sensor_data_types[j].find("image") != std::string::npos) {
                            sensor.image_data_offset = data_configs[j].first;
                            sensor.image_data_size = data_configs[j].second;
                            found_image = true;
                            std::cout << "  Using 'image' type for visualization (index " << j << ")" << std::endl;
                            break;
                        }
                    }
                    
                    // If no "image" type, look for "normal" type
                    if (!found_image) {
                        for (size_t j = 0; j < sensor_data_types.size() && j < data_configs.size(); j++) {
                            if (sensor_data_types[j].find("normal") != std::string::npos) {
                                sensor.image_data_offset = data_configs[j].first;
                                sensor.image_data_size = data_configs[j].second;
                                found_image = true;
                                std::cout << "  Using 'normal' type for visualization (index " << j << ")" << std::endl;
                                break;
                            }
                        }
                    }
                    
                    // If still no suitable type, look for "data" type as fallback
                    if (!found_image) {
                        for (size_t j = 0; j < sensor_data_types.size() && j < data_configs.size(); j++) {
                            if (sensor_data_types[j].find("data") != std::string::npos) {
                            sensor.image_data_offset = data_configs[j].first;
                            sensor.image_data_size = data_configs[j].second;
                            found_image = true;
                            std::cout << "  Using 'data' type for visualization (index " << j 
                                          << ") - prefer 'image' for better visualization" << std::endl;
                            break;
                        }
                        }
                    }
                    
                    // If we found image data, add this sensor
                    if (found_image && sensor.width > 0 && sensor.height > 0) {
                        // Allocate image buffer (RGB format for MuJoCo rendering)
                        sensor.scaled_width = sensor.width * display_scale_;
                        sensor.scaled_height = sensor.height * display_scale_;
                        sensor.image_buffer.reset(new unsigned char[sensor.scaled_width * sensor.scaled_height * 3]);
                        
                        depth_cameras_.push_back(std::move(sensor));
                        
                        std::cout << "Depth camera visualizer initialized: " << sensor_name
                                  << " (" << sensor.width << "x" << sensor.height 
                                  << " -> " << sensor.scaled_width << "x" << sensor.scaled_height << ")" << std::endl;
                    } else {
                        std::cout << "  No valid image data found for this sensor" << std::endl;
                    }
                }
            }
        }
        
        if (depth_cameras_.empty()) {
            std::cout << "No depth cameras found for visualization" << std::endl;
        }
        
        initialized_ = true;
    }
    
    void update_and_render(mjData* d) {
        if (!initialized_ || !sim_ || !d) return;
        if (depth_cameras_.empty()) return;
        
        // Position images starting from bottom-left
        int current_x = 10;
        int current_y = 10;
        
        for (size_t idx = 0; idx < depth_cameras_.size(); idx++) {
            auto& sensor = depth_cameras_[idx];
            
            // Get image data from sensor (data is mjtNum which is double)
            mjtNum* data_ptr = d->sensordata + sensor.data_adr + sensor.image_data_offset;
            
            // Convert depth data to RGB and scale
            unsigned char* img_ptr = sensor.image_buffer.get();
            int scaled_width = sensor.scaled_width;
            int scaled_height = sensor.scaled_height;
            int scale = scaled_width / sensor.width;
            
            // Optimized nearest-neighbor scaling with vertical flip
            for (int dst_row = 0; dst_row < scaled_height; dst_row++) {
                int src_row = dst_row / scale;
                int flipped_row = scaled_height - 1 - dst_row;
                int src_row_offset = src_row * sensor.width;
                int dst_row_offset = flipped_row * scaled_width;
                
                for (int dst_col = 0; dst_col < scaled_width; dst_col++) {
                    int src_col = dst_col / scale;
                    
                    double val = data_ptr[src_row_offset + src_col];
                    val = val < 0.0 ? 0.0 : (val > 255.0 ? 255.0 : val);
                    unsigned char gray = static_cast<unsigned char>(val);
                    
                    int dst_idx = (dst_row_offset + dst_col) * 3;
                    img_ptr[dst_idx] = gray;
                    img_ptr[dst_idx + 1] = gray;
                    img_ptr[dst_idx + 2] = gray;
                }
            }
            
            mjrRect viewport;
            viewport.width = scaled_width;
            viewport.height = scaled_height;
            viewport.left = current_x;
            viewport.bottom = current_y;
            
            // Move to next position
            current_x += scaled_width + 10;
            
            // Create a copy of the image data for MuJoCo to render
            auto image_copy = std::make_unique<unsigned char[]>(sensor.scaled_width * sensor.scaled_height * 3);
            std::memcpy(image_copy.get(), sensor.image_buffer.get(), 
                       sensor.scaled_width * sensor.scaled_height * 3);
            
            // Add to user images (thread-safe)
            sim_->user_images_new_.emplace_back(viewport, std::move(image_copy));
        }
        
        if (!depth_cameras_.empty()) {
            sim_->newimagerequest = 1;
        }
    }

private:
    struct DepthCameraSensor {
        std::string name;
        int sensor_id;
        int data_adr;
        int plugin_id;
        int state_adr;
        int width;
        int height;
        int image_data_offset;
        int image_data_size;
        int scaled_width = 0;
        int scaled_height = 0;
        std::unique_ptr<unsigned char[]> image_buffer;
    };
    
    bool initialized_ = false;
    int display_scale_ = 2;
    mujoco::Simulate* sim_ = nullptr;
    std::vector<DepthCameraSensor> depth_cameras_;
};
