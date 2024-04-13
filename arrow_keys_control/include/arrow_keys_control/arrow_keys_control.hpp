#ifndef ARROW_KEYS_CONTROL_HPP_
#define ARROW_KEYS_CONTROL_HPP_

#include <vector>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "cartesian_controller_base/ROS2VersionConfig.h"
#include <cartesian_controller_base/cartesian_controller_base.h>
#include <controller_interface/controller_interface.hpp>
//#include "Eigen3"
#include <Eigen/Dense>

namespace arrow_keys_control
{
    class ArrowKeysControl : public rclcpp::Node, public cartesian_controller_base::CartesianControllerBase 
    {
    public:
        ArrowKeysControl(rclcpp::NodeOptions options);

        virtual LifecycleNodeInterface::CallbackReturn on_init() override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
        virtual controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        void spin();
        using Base = cartesian_controller_base::CartesianControllerBase;
        std::string m_new_ft_sensor_ref;
        void setFtSensorReferenceFrame(const std::string & new_ref);
        bool readKeyNonBlocking(char& key);
        void handleCartesianInput();
        void moveAxis(int axis, double step, const std::string& message);

    private:

        bool key_thread_running_;

        rclcpp::TimerBase::SharedPtr timer_;
        ctrl::Vector6D m_cartesian_input;
        ctrl::Vector6D error;
        //ctrl::Vector6D old_position;
        //ctrl::Vector6D new_position;
        std::string m_ft_sensor_ref_link;
        KDL::Frame m_ft_sensor_transform;

        const double STEP_SIZE = 0.5;
        std::mutex mutex_;
    };
}

#endif  // ARROW_KEYS_CONTROL_HPP_

