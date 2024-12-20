#ifndef ELEVATOR_PKG__DRIVE_INTO_ELEVATOR_ACTION_HPP__
#define ELEVATOR_PKG__DRIVE_INTO_ELEVATOR_ACTION_HPP__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace elevator_pkg
{
        class DriveIntoElevatorActionServer: public rclcpp::Node
        {
        public:
                explicit DriveIntoElevatorActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
                ~DriveIntoElevatorActionServer();
        };
} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(elevator_pkg::DriveIntoElevatorActionServer)

#endif



