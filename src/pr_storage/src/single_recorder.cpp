#include "pr_storage/single_recorder.hpp"
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
    /**** SINGLE RECORDER COMPONENT ****/
    SingleRecorder::SingleRecorder(const rclcpp::NodeOptions & options)
    : Node("single_recorder", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("data_name", "joint_position");
        this->declare_parameter<std::string>("data_dir_path", "/home/avalera/pr_pc_ws/exp_data/exp_05_11_2020-16_33_14");

        this->get_parameter("data_name", data_name);
        this->get_parameter("data_dir_path", data_dir_path);

        file_path = data_dir_path + "/" + data_name + ".txt";

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "/sub",
                10,
                std::bind(&SingleRecorder::topic_callback, this, _1)
        );

        file.open(file_path);

        if(!file)
            RCLCPP_ERROR(this->get_logger(), "Could not open the file: " + file_path);
	    else 
	        RCLCPP_INFO(this->get_logger(), "File opened: " + file_path);       
    
        file_header();
    }

    SingleRecorder::~SingleRecorder(){
	        RCLCPP_INFO(this->get_logger(), "Closing file: " + file_path);
	        file.close();
    }

    void SingleRecorder::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr data_msg)
    {
        file << data_msg->header.stamp.sec << '\t' << data_msg->header.stamp.nanosec;
           
            for(int i=0; i<(long)data_msg->data.size(); i++){
                file << '\t' << data_msg->data[i];
            }

            file << std::endl;

        /*RCLCPP_INFO(this->get_logger(), 
            "Received from " + data_name + " '%f %f %f %f'", 
                                             data_msg->data[0], 
                                             data_msg->data[1], 
                                             data_msg->data[2], 
                                             data_msg->data[3]);*/
    }

    void SingleRecorder::file_header()
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
RCLCPP_COMPONENTS_REGISTER_NODE(pr_storage::SingleRecorder)