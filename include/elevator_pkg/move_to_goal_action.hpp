#ifndef ELEVATOR_PKG__MOVE_TO_GOAL_ACTION_HPP__
#define ELEVATOR_PKG__MOVE_TO_GOAL_ACTION_HPP__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "aruco_msgs/msg/pose_with_id.hpp"
#include "capella_ros_msg/action/move_to_goal.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "elevator_pkg/magic_enum.hpp"
#include "angles/angles.h"

#include <thread>

namespace elevator_pkg
{
        typedef enum {
                ANGLE_TO_GOAL,
                MOVE_TO_GOAL,
                GOAL_ANGLE,
                GOAL_SUCCESS
        }ROBOTSTATE;
        
        class MoveToGoalActionServer: public rclcpp::Node
        {
        public:
                using MoveToGoal = capella_ros_msg::action::MoveToGoal;
                using GoalHandleMoveToGoal = rclcpp_action::ServerGoalHandle<MoveToGoal>;
                
                explicit MoveToGoalActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
                ~MoveToGoalActionServer();
                void init_params();
                template <typename T>void declare_and_get_parameter(const std::string & param_name, const T& default_value, T& param_save);

                // parameters
                int timeout_{300};
                float linear_x_max_;
                float angular_max_;
                std::vector<double> acceleration_;  // std::vector<float>类型在get_parameter()时，自动变成std::vector<double>，导致类型冲突。
                float thre_angle_to_goal_;
                float tolerance_xy_, tolerance_yaw;
                int hz_;
                
                // action server
                rclcpp_action::Server<MoveToGoal>::SharedPtr action_server_;
                rclcpp_action::GoalResponse handle_goal(
                        const rclcpp_action::GoalUUID & uuid,
                        std::shared_ptr<const MoveToGoal::Goal> goal
                );
                rclcpp_action::CancelResponse handle_cancel(
                        const std::shared_ptr<GoalHandleMoveToGoal> goal_handle
                );
                void handle_accepted(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle);
                void execute(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle);

                float generate_new_speed(const float & speed_current, const float & speed_max, const float& acc, const float & dist, double speed_time_last);
                
                // subs
                rclcpp::Subscription<aruco_msgs::msg::PoseWithId>::SharedPtr marker_pose_sub_;
                void marker_pose_sub_callback_(aruco_msgs::msg::PoseWithId::SharedPtr msg);

                // pubs
                rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
                
                int marker_id;
                float goal_x, goal_y, goal_yaw;
                float robot_x, robot_y, robot_yaw;

                tf2::Transform tf_rotation;
                double action_start_time;

                ROBOTSTATE robot_state;
                ROBOTSTATE robot_state_last;

                geometry_msgs::msg::Twist cmd_vel_;

        };

} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(elevator_pkg::MoveToGoalActionServer)

#endif



