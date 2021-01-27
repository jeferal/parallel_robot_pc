#ifndef PR_STORAGE__MOCAP_RECORDER_HPP_
#define PR_STORAGE__MOCAP_RECORDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_mocap.hpp"

#include <fstream>
#include <vector>
#include <string>

namespace pr_storage
{
    class MocapRecorder : public rclcpp::Node
    {
        public:
            //PR_STORAGE_PUBLIC
            explicit MocapRecorder(const rclcpp::NodeOptions & options);
            ~MocapRecorder();
        protected:
            void topic_callback(const pr_msgs::msg::PRMocap::SharedPtr data_msg);
            void file_header();
        private:
            rclcpp::Subscription<pr_msgs::msg::PRMocap>::SharedPtr subscription_;
            std::string data_name;
            std::string data_dir_path, file_path;
            std::ofstream file;
    };
}

#endif // PR_STORAGE__SINGLE_RECORDER_HPP_