#include "pr_storage/number_recorder.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"

using std::placeholders::_1;


namespace pr_storage
{
    /**** NUMBER RECORDER COMPONENT ****/
    NumberRecorder::NumberRecorder(const rclcpp::NodeOptions & options)
    : Node("number_recorder", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("data_name", "joint_position");
        this->declare_parameter<std::string>("data_dir_path", "/home/avalera/pr_pc_ws/exp_data/exp_05_11_2020-16_33_14");

        this->get_parameter("data_name", data_name);
        this->get_parameter("data_dir_path", data_dir_path);

        file_path = data_dir_path + "/" + data_name + ".txt";

        subscription_ = this->create_subscription<pr_msgs::msg::PRFloatH>(
            "/sub",
                10,
                std::bind(&NumberRecorder::topic_callback, this, _1)
        );

        file.open(file_path);

        if(!file)
            RCLCPP_ERROR(this->get_logger(), "Could not open the file: " + file_path);
	    else 
	        RCLCPP_INFO(this->get_logger(), "File opened: " + file_path);       
    
        file_header();
    }

    NumberRecorder::~NumberRecorder(){
	        RCLCPP_INFO(this->get_logger(), "Closing the file: " + file_path);
	        file.close();
    }

    void NumberRecorder::topic_callback(const pr_msgs::msg::PRFloatH::SharedPtr data_msg)
    {
        file << data_msg->current_time.sec << '\t' << data_msg->current_time.nanosec;
           
        file << '\t' << data_msg->data;

        file << std::endl;
            
    }

    void NumberRecorder::file_header()
    {
        file << "time_sec\t" << "time_nanosec"; 

        file << '\t' << data_name;

        file << std::endl;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_storage::NumberRecorder)