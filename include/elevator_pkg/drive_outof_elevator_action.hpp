#ifndef ELEVATOR_PKG__DRIVE_OUTOF_ELEVATOR_ACTION_HPP__
#define ELEVATOR_PKG__DRIVE_OUTOF_ELEVATOR_ACTION_HPP__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace elevator_pkg
{
        class DriveOutofElevatorActionServer: public rclcpp::Node
        {
        public:
                explicit DriveOutofElevatorActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
                ~DriveOutofElevatorActionServer();
        };
} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(elevator_pkg::DriveOutofElevatorActionServer)

#endif



