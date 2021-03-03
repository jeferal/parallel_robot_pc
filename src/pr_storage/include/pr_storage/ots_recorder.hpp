#ifndef PR_STORAGE__OTS_RECORDER_HPP_
#define PR_STORAGE__OTS_RECORDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/prots.hpp"

#include <fstream>
#include <vector>
#include <string>

namespace pr_storage
{
    class OTSRecorder : public rclcpp::Node
    {
        public:
            //PR_STORAGE_PUBLIC
            explicit OTSRecorder(const rclcpp::NodeOptions & options);
            ~OTSRecorder();
        protected:
            void topic_callback(const pr_msgs::msg::PROTS::SharedPtr data_msg);
            void file_header();
        private:
            rclcpp::Subscription<pr_msgs::msg::PROTS>::SharedPtr subscription_;
            std::string data_name;
            std::string data_dir_path, file_path;
            std::ofstream file;
    };
}

#endif // PR_STORAGE__OTS_RECORDER_HPP_