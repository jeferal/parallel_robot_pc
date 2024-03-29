#include "pr_storage/ots_recorder.hpp"

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
    /**** OTS RECORDER COMPONENT ****/
    OTSRecorder::OTSRecorder(const rclcpp::NodeOptions & options)
    : Node("ots_recorder", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("data_name_1", "ang_ots");
        this->declare_parameter<std::string>("data_name_2", "sol_ots");
        this->declare_parameter<std::string>("data_dir_path", "/home/avalera/pr_pc_ws/exp_data/exp_28_01_2021-13_07_54");

        this->get_parameter("data_name_1", data_name);
        this->get_parameter("data_dir_path", data_dir_path);

        file_path = data_dir_path + "/" + data_name + ".txt";

        subscription_ = this->create_subscription<pr_msgs::msg::PROTS>(
            "/sub",
                10,
                std::bind(&OTSRecorder::topic_callback, this, _1)
        );

        file.open(file_path);

        if(!file)
            RCLCPP_ERROR(this->get_logger(), "Could not open the file: " + file_path);
	    else 
	        RCLCPP_INFO(this->get_logger(), "File opened: " + file_path);       
    
        file_header();
    }

    OTSRecorder::~OTSRecorder(){
	        RCLCPP_INFO(this->get_logger(), "Closing file: " + file_path);
	        file.close();
    }

    void OTSRecorder::topic_callback(const pr_msgs::msg::PROTS::SharedPtr ots_msg)
    {
        file << ots_msg->current_time.sec << '\t' << ots_msg->current_time.nanosec;
           
            for(int i=0; i<(long)ots_msg->ots_ang.size(); i++){
                file << '\t' << ots_msg->ots_ang[i];
            }

        file << std::endl;

    }

    void OTSRecorder::file_header()
    {
        file << "time_sec\t" << "time_nanosec"; 

        for(int i=0; i<6; i++)
            file << '\t' << data_name << "_" << i;

        file << std::endl;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_storage::OTSRecorder)
