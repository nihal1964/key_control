#include "/home/nihal/hc_ur10e/src/arrow_keys_control/include/arrow_keys_control/arrow_keys_control.hpp"
#include <iostream>
#include <chrono>
//#include <atomic>
#include <csignal>
//#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cmath>
#include <mutex>
#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"

#define KEYCODE_Q 0x71
#define KEYCODE_A 0x61
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_E 0x65
#define KEYCODE_D 0x64
#define KEYCODE_X 0x78

namespace arrow_keys_control
{
    ArrowKeysControl::ArrowKeysControl(rclcpp::NodeOptions options)
        : Node("arrow_keys_control_node", std::move(options)),
          Base::CartesianControllerBase(),
          key_thread_running_(true),
          m_cartesian_input(ctrl::Vector6D::Zero()),
          mutex_()
    {
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ArrowKeysControl::on_init()
    {
        try {
            const auto ret = Base::on_init();
            if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
                RCLCPP_ERROR(get_logger(), "Failed to initialize base class");
                return ret;
            }
            auto_declare<std::string>("ft_sensor_ref_link", "");
            auto_declare("timer", "true");
            auto_declare("step_size", STEP_SIZE);
        
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in on_init(): %s", e.what());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ArrowKeysControl::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        try {
            const auto ret = Base::on_configure(previous_state);
            if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
                RCLCPP_ERROR(get_logger(), "Failed to configure base class");
                return ret;
            }

            // Declare parameters
            m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
            if (!Base::robotChainContains(m_ft_sensor_ref_link))
            {
                RCLCPP_ERROR(get_logger(), "%s is not part of the kinematic chain from %s to %s", 
                              m_ft_sensor_ref_link.c_str(), Base::m_robot_base_link.c_str(), Base::m_end_effector_link.c_str());
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }

            // Make sure sensor wrenches are interpreted correctly
            setFtSensorReferenceFrame(Base::m_end_effector_link);
            timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArrowKeysControl::handleCartesianInput, this));
            this->declare_parameter("step_size", STEP_SIZE);
        
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in on_configure(): %s", e.what());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ArrowKeysControl::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        try {
            const auto ret = Base::on_activate(previous_state);
            if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
                RCLCPP_ERROR(get_logger(), "Failed to activate base class");
                return ret;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in on_activate(): %s", e.what());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ArrowKeysControl::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        try {
            const auto ret = Base::on_deactivate(previous_state);
            if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
                RCLCPP_ERROR(get_logger(), "Failed to deactivate base class");
                return ret;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in on_deactivate(): %s", e.what());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ArrowKeysControl::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
        try {
            Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);
            (void)time;
            (void)period;
            auto internal_period = std::chrono::seconds(1) / 50;

            // Acquire mutex before accessing error
            std::lock_guard<std::mutex> lock(mutex_);

            // Turn Cartesian error into joint motion
            Base::computeJointControlCmds(error, std::chrono::milliseconds(100));;

            // Write final commands to the hardware interface
            Base::writeJointControlCmds();
            return controller_interface::return_type::OK;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in update(): %s", e.what());
            return controller_interface::return_type::ERROR;
        }
    }

    void ArrowKeysControl::spin() {
        try {
            rclcpp::spin(this->get_node_base_interface());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in spin(): %s", e.what());
        }
    }

    void ArrowKeysControl::setFtSensorReferenceFrame(const std::string & new_ref)
    {
        try {
            // Compute static transform from the force torque sensor to the new reference
            // frame of interest.
            m_new_ft_sensor_ref = new_ref;

            // Joint positions should cancel out, i.e. it doesn't matter as long as they
            // are the same for both transformations.
            auto jnts = Base::m_ik_solver->getPositions();

            KDL::Frame sensor_ref;
            Base::m_forward_kinematics_solver->JntToCart(jnts, sensor_ref, m_ft_sensor_ref_link);

            KDL::Frame new_sensor_ref;
            Base::m_forward_kinematics_solver->JntToCart(jnts, new_sensor_ref, m_new_ft_sensor_ref);

            m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in setFtSensorReferenceFrame(): %s", e.what());
        }
    }

    bool ArrowKeysControl::readKeyNonBlocking(char& key)
    {
        try {
            termios oldt, newt;
            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);

            int bytes_waiting;
            ioctl(STDIN_FILENO, FIONREAD, &bytes_waiting);
            if (bytes_waiting > 0) {
                std::cin >> key;
                tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
                return true;
            }

            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in readKeyNonBlocking(): %s", e.what());
            return false;
        }
    }

    void ArrowKeysControl::handleCartesianInput()
    {
        try {
            puts("Press arrow keys to set the direction");
            puts("---------------------------");
            puts("Press 'x' to quit");

            char key;
            while (key_thread_running_ && rclcpp::ok()) {
                if (readKeyNonBlocking(key)) {
                    switch(key) {
                        case KEYCODE_W:
                            moveAxis(0, STEP_SIZE, "X-axis motion increased");
                            break;
                        case KEYCODE_S:
                            moveAxis(0, -STEP_SIZE, "X-axis motion decreased");
                            break;
                        case KEYCODE_A:
                            moveAxis(1, STEP_SIZE, "Y-axis motion increased");
                            break;
                        case KEYCODE_D:
                            moveAxis(1, -STEP_SIZE, "Y-axis motion decreased");
                            break;
                        case KEYCODE_Q:
                            moveAxis(2, STEP_SIZE, "Z-axis motion increased");
                            break;
                        case KEYCODE_E:
                            moveAxis(2, -STEP_SIZE, "Z-axis motion decreased");
                            break;
                        case KEYCODE_X:
                            // Deactivate before shutting down
                            RCLCPP_INFO(get_logger(), "Quitting...");
                            on_deactivate(rclcpp_lifecycle::State()); // Call on_deactivate() before shutting down
                            rclcpp::shutdown();
                            break;
                        default:
                            RCLCPP_WARN(get_logger(), "Invalid key pressed");
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in handleCartesianInput(): %s", e.what());
        }
    }


    void ArrowKeysControl::moveAxis(int axis, double step, const std::string& message)
    {
        try {
            std::lock_guard<std::mutex> lock(mutex_); // Lock mutex
            auto old_position = m_cartesian_input[axis]; // Get the old position
            m_cartesian_input[axis] += step; // Update the position based on the step size
            auto new_position = m_cartesian_input[axis]; // Get the new position
            error[axis] = new_position - old_position;
            Base::computeJointControlCmds(error, std::chrono::milliseconds(100));
            Base::writeJointControlCmds();
            RCLCPP_DEBUG(get_logger(), "%s", message.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in moveAxis(): %s", e.what());
        }
    }

    void exitSigHandler(int sig)
    {
        (void)sig;
        RCLCPP_INFO(rclcpp::get_logger("arrow_keys_control"), "Received SIGINT, shutting down...");
        rclcpp::shutdown();
    }

} // namespace arrow_keys_control

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        rclcpp::NodeOptions node_options;
        node_options.use_intra_process_comms(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        auto node = std::make_shared<rclcpp::Node>("arrow_keys_control", node_options);

        // Create the ArrowKeysControl node
        auto arrow_keys_control = std::make_shared<arrow_keys_control::ArrowKeysControl>(node->get_node_options());
        arrow_keys_control->configure();

        // Trigger the activate state transition
        arrow_keys_control->on_activate(rclcpp_lifecycle::State());

        // Register SIGINT handler
        signal(SIGINT, arrow_keys_control::exitSigHandler);

        // Spin the node
        arrow_keys_control->spin();

        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception occurred in main(): %s", e.what());
        return -1;
    }
}

