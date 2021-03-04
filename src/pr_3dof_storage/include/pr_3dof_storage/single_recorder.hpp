#ifndef PR_3DOF_STORAGE__SINGLE_RECORDER_HPP_
#define PR_3DOF_STORAGE__SINGLE_RECORDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_3dof_msgs/msg/pr_array_h.hpp"

#include <fstream>
#include <vector>
#include <string>

namespace pr_3dof_storage
{
    class SingleRecorder : public rclcpp::Node
    {
        public:
            //PR_3DOF_STORAGE_PUBLIC
            explicit SingleRecorder(const rclcpp::NodeOptions & options);
            ~SingleRecorder();
        protected:
            void topic_callback(const pr_3dof_msgs::msg::PRArrayH::SharedPtr data_msg);
            void file_header();
        private:
            rclcpp::Subscription<pr_3dof_msgs::msg::PRArrayH>::SharedPtr subscription_;
            std::string data_name;
            std::string data_dir_path, file_path;
            std::ofstream file;
    };
}

#endif // PR_3DOF_STORAGE__SINGLE_RECORDER_HPP_