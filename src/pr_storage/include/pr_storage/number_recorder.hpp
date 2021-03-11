#ifndef PR_STORAGE__NUMBER_RECORDER_HPP_
#define PR_STORAGE__NUMBER_RECORDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"

#include <fstream>
#include <vector>
#include <string>

namespace pr_storage
{
    class NumberRecorder : public rclcpp::Node
    {
        public:
            //PR_STORAGE_PUBLIC
            explicit NumberRecorder(const rclcpp::NodeOptions & options);
            ~NumberRecorder();
        protected:
            void topic_callback(const pr_msgs::msg::PRFloatH::SharedPtr data_msg);
            void file_header();
        private:
            rclcpp::Subscription<pr_msgs::msg::PRFloatH>::SharedPtr subscription_;
            std::string data_name;
            std::string data_dir_path, file_path;
            std::ofstream file;
    };
}

#endif // PR_STORAGE__SINGLE_RECORDER_HPP_