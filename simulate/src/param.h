#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <map>
#include <string>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    bool enable_ray_array = false;
    bool enable_odom = false;
    bool enable_odom_sub = false;
    std::string odom_sub_mode = "odom";
    std::string odom_sub_topic = "/external_odom";
    std::string odom_sub_tf_source_frame = "odom";
    std::string odom_sub_tf_target_frame = "base_link";
    bool enable_gridmap = false;
    bool enable_depth_visualizer = false;

    std::string raycaster_output_format = "pointcloud";
    bool raycaster_flatten_xyz = true;
    bool raycaster_zero_mean = false;

    struct RaycasterSensorConfig {
        std::string output_format;
        bool flatten_xyz = true;
        bool zero_mean = false;
        float offset = 0.0f;
        std::string replace_nan;
        float max_distance = 10.0f;
        std::string distance_type;
    };
    std::map<std::string, RaycasterSensorConfig> raycaster_sensors;

    double publisher_frequency = 50.0;
    double depth_visualizer_frequency = 10.0;
    int depth_visualizer_scale = 2;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();

            if (cfg["enable_ray_array"]) {
                enable_ray_array = cfg["enable_ray_array"].as<bool>();
            }
            if (cfg["enable_odom"]) {
                enable_odom = cfg["enable_odom"].as<bool>();
            }
            if (cfg["enable_odom_sub"]) {
                enable_odom_sub = cfg["enable_odom_sub"].as<bool>();
            }
            if (cfg["odom_sub_mode"]) {
                odom_sub_mode = cfg["odom_sub_mode"].as<std::string>();
            }
            if (cfg["odom_sub_topic"]) {
                odom_sub_topic = cfg["odom_sub_topic"].as<std::string>();
            }
            if (cfg["odom_sub_tf_source_frame"]) {
                odom_sub_tf_source_frame = cfg["odom_sub_tf_source_frame"].as<std::string>();
            }
            if (cfg["odom_sub_tf_target_frame"]) {
                odom_sub_tf_target_frame = cfg["odom_sub_tf_target_frame"].as<std::string>();
            }
            if (cfg["enable_gridmap"]) {
                enable_gridmap = cfg["enable_gridmap"].as<bool>();
            }
            if (cfg["enable_depth_visualizer"]) {
                enable_depth_visualizer = cfg["enable_depth_visualizer"].as<bool>();
            }
            if (cfg["raycaster_output_format"]) {
                raycaster_output_format = cfg["raycaster_output_format"].as<std::string>();
            }
            if (cfg["raycaster_flatten_xyz"]) {
                raycaster_flatten_xyz = cfg["raycaster_flatten_xyz"].as<bool>();
            }
            if (cfg["raycaster_zero_mean"]) {
                raycaster_zero_mean = cfg["raycaster_zero_mean"].as<bool>();
            }
            if (cfg["raycaster_sensors"]) {
                raycaster_sensors.clear();
                for (const auto& sensor : cfg["raycaster_sensors"]) {
                    std::string name = sensor.first.as<std::string>();
                    RaycasterSensorConfig sensor_cfg;

                    if (sensor.second["output_format"]) {
                        sensor_cfg.output_format = sensor.second["output_format"].as<std::string>();
                    }
                    if (sensor.second["flatten_xyz"]) {
                        sensor_cfg.flatten_xyz = sensor.second["flatten_xyz"].as<bool>();
                    }
                    if (sensor.second["zero_mean"]) {
                        sensor_cfg.zero_mean = sensor.second["zero_mean"].as<bool>();
                    }
                    if (sensor.second["offset"]) {
                        sensor_cfg.offset = sensor.second["offset"].as<float>();
                    }
                    if (sensor.second["replace_nan"]) {
                        sensor_cfg.replace_nan = sensor.second["replace_nan"].as<std::string>();
                    }
                    if (sensor.second["max_distance"]) {
                        sensor_cfg.max_distance = sensor.second["max_distance"].as<float>();
                    }
                    if (sensor.second["distance_type"]) {
                        sensor_cfg.distance_type = sensor.second["distance_type"].as<std::string>();
                    }

                    raycaster_sensors[name] = sensor_cfg;
                }
            }
            if (cfg["publisher_frequency"]) {
                publisher_frequency = cfg["publisher_frequency"].as<double>();
            }
            if (cfg["depth_visualizer_frequency"]) {
                depth_visualizer_frequency = cfg["depth_visualizer_frequency"].as<double>();
            }
            if (cfg["depth_visualizer_scale"]) {
                depth_visualizer_scale = cfg["depth_visualizer_scale"].as<int>();
                if (depth_visualizer_scale < 1) depth_visualizer_scale = 1;
                if (depth_visualizer_scale > 4) depth_visualizer_scale = 4;
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}
