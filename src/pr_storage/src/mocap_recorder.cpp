#include "pr_storage/mocap_recorder.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;


namespace pr_storage
{
    /**** MOCAP RECORDER COMPONENT ****/
    MocapRecorder::MocapRecorder(const rclcpp::NodeOptions & options)
    : Node("mocap_recorder", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("data_name", "x_mocap");
        this->declare_parameter<std::string>("data_dir_path", "/home/avalera/pr_pc_ws/exp_data/exp_28_01_2021-13_07_54");

        this->get_parameter("data_name", data_name);
        this->get_parameter("data_dir_path", data_dir_path);

        file_path = data_dir_path + "/" + data_name + ".txt";

        subscription_ = this->create_subscription<pr_msgs::msg::PRMocap>(
            "/sub",
                10,
                std::bind(&MocapRecorder::topic_callback, this, _1)
        );

        file.open(file_path);

        if(!file)
            RCLCPP_ERROR(this->get_logger(), "Could not open the file: " + file_path);
	    else 
	        RCLCPP_INFO(this->get_logger(), "File opened: " + file_path);       
    
        file_header();
    }

    MocapRecorder::~MocapRecorder(){
	        RCLCPP_INFO(this->get_logger(), "Closing file: " + file_path);
	        file.close();
    }

    void MocapRecorder::topic_callback(const pr_msgs::msg::PRMocap::SharedPtr mocap_msg)
    {
        file << mocap_msg->current_time.sec << '\t' << mocap_msg->current_time.nanosec;
           
            for(int i=0; i<(long)mocap_msg->x_coord.data.size(); i++){
                file << '\t' << mocap_msg->x_coord.data[i];
            }

        file << '\t' << mocap_msg->error << '\t' << mocap_msg->latency;

        file << std::endl;

        /*RCLCPP_INFO(this->get_logger(), 
            "Received from " + data_name + " '%f %f %f %f'", 
                                             data_msg->data[0], 
                                             data_msg->data[1], 
                                             data_msg->data[2], 
                                             data_msg->data[3]);*/
    }

    void MocapRecorder::file_header()
    {
        file << "time_sec\t" << "time_nanosec"; 

        for(int i=0; i<4; i++)
            file << '\t' << data_name << "_" << i;

        file << "\terror\tlatency"; 

        file << std::endl;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_storage::MocapRecorder)