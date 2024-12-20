#include "elevator_pkg/drive_outof_elevator_action.hpp"

namespace elevator_pkg{

DriveOutofElevatorActionServer::DriveOutofElevatorActionServer(const rclcpp::NodeOptions & options)
        : Node("drive_outof_elevator_action_server", options)
{
        RCLCPP_INFO(get_logger(), "drive_outof_elevator_action_server started.");
}

DriveOutofElevatorActionServer::~DriveOutofElevatorActionServer()
{
        RCLCPP_INFO(get_logger(), "drive_outof_elevator_action_server destructor.");
}

} // end of namespace

