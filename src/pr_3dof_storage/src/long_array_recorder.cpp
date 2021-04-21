#include "pr_3dof_storage/long_array_recorder.hpp"
#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;


namespace pr_3dof_storage
{
    /**** 3DOF LONG ARRAY RECORDER COMPONENT ****/
    LongArrayRecorder::LongArrayRecorder(const rclcpp::NodeOptions & options)
    : Node("long_array_recorder", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("data_name", "joint_position");
        this->declare_parameter<std::string>("data_dir_path", "/home/avalera/pr_pc_ws/exp_data/exp_05_11_2020-16_33_14");

        this->get_parameter("data_name", data_name);
        this->get_parameter("data_dir_path", data_dir_path);

        file_path = data_dir_path + "/" + data_name + ".txt";

        subscription_ = this->create_subscription<pr_3dof_msgs::msg::PRLongArrayH>(
            "/sub",
                10,
                std::bind(&LongArrayRecorder::topic_callback, this, _1)
        );

        file.open(file_path);

        if(!file)
            RCLCPP_ERROR(this->get_logger(), "Could not open the file: " + file_path);
	    else 
	        RCLCPP_INFO(this->get_logger(), "File opened: " + file_path);       
    
        file_header();
    }

    LongArrayRecorder::~LongArrayRecorder(){
	        RCLCPP_INFO(this->get_logger(), "Closing the file: " + file_path);
	        file.close();
    }

    void LongArrayRecorder::topic_callback(const pr_3dof_msgs::msg::PRLongArrayH::SharedPtr data_msg)
    {
        file << data_msg->current_time.sec << '\t' << data_msg->current_time.nanosec;
           
            for(int i=0; i<(int)data_msg->data.size(); i++){
                file << '\t' << data_msg->data[i];
            }

            file << std::endl;
            
    }

    void LongArrayRecorder::file_header()
    {
        file << "time_sec\t" << "time_nanosec"; 

        for(int i=0; i<4; i++)
            file << '\t' << data_name << "_" << i;

        file << std::endl;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_3dof_storage::LongArrayRecorder)