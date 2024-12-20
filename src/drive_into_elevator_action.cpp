#include "elevator_pkg/drive_into_elevator_action.hpp"

namespace elevator_pkg{

DriveIntoElevatorActionServer::DriveIntoElevatorActionServer(const rclcpp::NodeOptions & options)
        : Node("drive_into_elevator_action_server", options)
{
        RCLCPP_INFO(get_logger(), "drive_into_elevator_action_server started.");
}

DriveIntoElevatorActionServer::~DriveIntoElevatorActionServer()
{
        RCLCPP_INFO(get_logger(), "drive_into_elevator_action_server destructor.");
}

} // end of namespace

