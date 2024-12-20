#include "elevator_pkg/move_to_goal_action.hpp"

namespace elevator_pkg{

using namespace std::placeholders;

MoveToGoalActionServer::MoveToGoalActionServer(const rclcpp::NodeOptions & options)
        : Node("move_to_goal_action_server", options)
{
        RCLCPP_INFO(get_logger(), "move_to_goal_action_server started.");
        
        tf_rotation.setIdentity();
        tf2::Quaternion q_rotation;
        q_rotation.setRPY(0.0, -M_PI / 2.0, 0.0);
        tf_rotation.setRotation(q_rotation);

        init_params();

        action_server_ = rclcpp_action::create_server<MoveToGoal>(
                this,
                "move_to_goal",
                std::bind(&MoveToGoalActionServer::handle_goal, this, _1, _2),
                std::bind(&MoveToGoalActionServer::handle_cancel, this, _1),
                std::bind(&MoveToGoalActionServer::handle_accepted, this, _1)
        );
}

MoveToGoalActionServer::~MoveToGoalActionServer()
{
        RCLCPP_INFO(get_logger(), "move_to_goal_action_server destructor.");
}

void MoveToGoalActionServer::init_params()
{
        declare_and_get_parameter<int>("timeout", 100, timeout_);
        declare_and_get_parameter<float>("linear_x_max", 0.1, linear_x_max_);
        declare_and_get_parameter<float>("angular_max", 0.1, angular_max_);
        declare_and_get_parameter<float>("thre_angle_to_goal", 0.1, thre_angle_to_goal_);
        declare_and_get_parameter<std::vector<double>>("acceleration", std::vector<double>(), acceleration_);
        if (acceleration_.size() != 2)
        {
                RCLCPP_ERROR(get_logger(), "Error params for acceleration.");
        }
        declare_and_get_parameter<float>("tolerace_xy", 0.1, tolerance_xy_);
        declare_and_get_parameter<float>("tolerance_yaw", 0.05, tolerance_yaw);
        declare_and_get_parameter<int>("hz", 5, hz_);
}

template <typename T>
void MoveToGoalActionServer::declare_and_get_parameter(const std::string& param_name, const T& default_value, T& param_save)
{
        T value;
        declare_parameter<T>(param_name, default_value);
        value = get_parameter(param_name).get_value<T>();
        param_save = value;
}

rclcpp_action::GoalResponse MoveToGoalActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToGoal::Goal> goal)
{
        action_start_time = now().seconds();
        (void) uuid;
        robot_state = ROBOTSTATE::ANGLE_TO_GOAL;
        robot_state_last = ROBOTSTATE::ANGLE_TO_GOAL;
        auto pose = goal->pose;
        tf2::Transform tf;
        tf2::convert(pose, tf);

        goal_x = tf.getOrigin().getX();
        goal_y = tf.getOrigin().getY();
        goal_yaw = tf2::getYaw(tf.getRotation());
        RCLCPP_INFO(get_logger(), "Received a goal request for pose=> x: %f, y: %f, yaw: %f", goal_x, goal_y, goal_yaw);

        auto callback_type = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options = rclcpp::SubscriptionOptions();
        options.callback_group = callback_type;

        marker_pose_sub_ = this->create_subscription<aruco_msgs::msg::PoseWithId>(
                "pose_with_id", rclcpp::QoS(20).reliable(), 
                std::bind(&MoveToGoalActionServer::marker_pose_sub_callback_, this, _1), options);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(5).best_effort());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveToGoalActionServer::handle_cancel(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
{
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Cancel subscription on topic /pose_with_id");
        marker_pose_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Cancel publisher on topic /cmd_vel");
        cmd_vel_pub_.reset();    
        return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveToGoalActionServer::handle_accepted(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
{
        std::thread{std::bind(&MoveToGoalActionServer::execute, this, _1), goal_handle}.detach();
}

void MoveToGoalActionServer::execute(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
{
        auto result = std::make_shared<MoveToGoal::Result>();
        auto feed_back = std::make_shared<MoveToGoal::Feedback>();

        auto wall_rate_pub_cmd = rclcpp::WallRate(static_cast<double>(hz_));        

        float dist_xy, dist_yaw, theta_to_goal;

        double speed_time_last = this->now().seconds();
        
        RCLCPP_INFO(get_logger(), "Begin turn robot's pose.");
        RCLCPP_INFO(get_logger(), "state => ANAGLE_TO_GOAL. linear: %f, anglular: %f", cmd_vel_.linear.x, cmd_vel_.angular.z);
        while(true)
        {
                theta_to_goal = std::atan2(goal_y - robot_y, goal_x - robot_x);
                dist_yaw = angles::shortest_angular_distance(robot_yaw, theta_to_goal);
                dist_xy = std::hypot(robot_x - goal_x, robot_y - goal_y);

                switch(robot_state)
                {
                        case ROBOTSTATE::ANGLE_TO_GOAL:
                        {
                                cmd_vel_.linear.x = 0.0;
                                if (std::abs(dist_yaw) >= thre_angle_to_goal_)
                                {
                                        cmd_vel_.angular.z = generate_new_speed(cmd_vel_.angular.z, angular_max_, acceleration_[1], dist_yaw,speed_time_last);
                                }
                                else
                                {
                                        cmd_vel_.angular.z = 0.0;
                                        robot_state = ROBOTSTATE::MOVE_TO_GOAL;
                                }
                                speed_time_last = now().seconds();
                                break;
                        }
                        case ROBOTSTATE::MOVE_TO_GOAL:
                        {
                                if (std::abs(dist_xy) < tolerance_xy_)
                                {
                                        robot_state = ROBOTSTATE::GOAL_ANGLE;
                                        cmd_vel_.linear.x = 0.0;
                                        cmd_vel_.angular.z = 0.0;
                                }
                                else
                                {
                                        cmd_vel_.linear.x = generate_new_speed(cmd_vel_.linear.x, linear_x_max_, acceleration_[0], dist_xy, speed_time_last);
                                        cmd_vel_.angular.z = dist_yaw;
                                }
                                speed_time_last = now().seconds();
                                break;
                        }
                        case ROBOTSTATE::GOAL_ANGLE:
                        {
                                cmd_vel_.linear.x = 0;
                                if (std::abs(dist_yaw) < tolerance_xy_)
                                {
                                        robot_state = ROBOTSTATE::GOAL_SUCCESS;
                                        cmd_vel_.angular.z = 0.0;
                                }
                                else
                                {
                                        cmd_vel_.angular.z = generate_new_speed(cmd_vel_.angular.z, angular_max_, acceleration_[1], dist_yaw,speed_time_last);
                                }
                                speed_time_last = now().seconds();
                                break;
                        }
                        case ROBOTSTATE::GOAL_SUCCESS:
                        {
                                cmd_vel_.linear.x = 0.0;
                                cmd_vel_.angular.z = 0.0;
                                break;
                        }
                }

                if (robot_state_last != robot_state)
                {
                        RCLCPP_INFO(get_logger(), "robot_state change to ", magic_enum::enum_name(robot_state));
                }

                if (robot_state == ROBOTSTATE::GOAL_SUCCESS)
                {
                        goal_handle->succeed(result);
                        break;
                }
                else if(this->get_clock()->now().seconds() - action_start_time > timeout_)
                {
                        RCLCPP_INFO(get_logger(), "Timeout, cancel the goal...");
                        goal_handle->canceled(result);
                }
                else
                {
                        wall_rate_pub_cmd.sleep();
                        cmd_vel_pub_->publish(cmd_vel_);
                        feed_back->distance_remaining = dist_xy;
                        geometry_msgs::msg::Point point;
                        point.x = robot_x;
                        point.y = robot_y;
                        point.z = 0.0;                        
                        feed_back->current_pose = point;
                        feed_back->current_yaw = robot_yaw;
                        feed_back->cmd_linear = cmd_vel_.linear.x;
                        feed_back->cmd_angular = cmd_vel_.angular.z;
                        goal_handle->publish_feedback(feed_back);
                }
        }
}

void MoveToGoalActionServer::marker_pose_sub_callback_(aruco_msgs::msg::PoseWithId::SharedPtr msg)
{
        tf2::Transform tf;
        tf2::convert(msg->pose.pose, tf);
        tf *= tf_rotation;
        robot_x = tf.getOrigin().getX();
        robot_y = tf.getOrigin().getY();
        robot_yaw = tf2::getYaw(tf.getRotation());
}

float MoveToGoalActionServer::generate_new_speed(const float & speed_current, const float & speed_max, const float& acc, const float & dist, double speed_time_last)
{
        float dist_thr = ((speed_max + 0.0) / 2.0) * ((speed_max - 0.0) / acc);  // (Vmax + Vmin) / 2.0 平均速度， （Vmax-Vmin)/acc 运行时间
        
        int sign;
        if (std::abs(dist) > dist_thr)
        {
                sign = 1;
        }
        else
        {
                sign = -1;
        }

        float delta_time = this->now().seconds() - speed_time_last;
        float speed_new = std::copysign((std::abs(speed_current) + acc * delta_time * sign), speed_current);
        return speed_new;
}

} // end of namespace

