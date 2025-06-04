#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>

#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"
#include "aurora/kinematics.h"
#include "aurora/kinematic_links.cpp"

MAKE_exchange_backend_state();
MAKE_exchange_marker_reports_depth();
MAKE_exchange_moveit_goal();
MAKE_exchange_moveit_plan();

class LunaticMoveIt : public rclcpp::Node {
public:
  LunaticMoveIt()
      : Node("lunatic_moveit"),
        group_(shared_from_this(), "excahauler_arm") {
    group_.setPoseReferenceFrame("frame_link");
    group_.setMaxVelocityScalingFactor(0.5);
  }

  void spin() {
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      process_goal();
      rate.sleep();
    }
  }

private:
  void process_goal() {
    const auto goal = exchange_moveit_goal.read();
    if (goal.percent <= 0.0f)
      return;

    if (goal.percent == last_goal_.percent && goal.origin == last_goal_.origin)
      return; // already processed

    RCLCPP_INFO(get_logger(), "MoveIt planning to %.2f %.2f %.2f", goal.origin.x,
                goal.origin.y, goal.origin.z);

    geometry_msgs::msg::Pose pose;
    pose.position.x = goal.origin.x;
    pose.position.y = goal.origin.y;
    pose.position.z = goal.origin.z;

    Eigen::Matrix3d R;
    R.col(0) << goal.X.x, goal.X.y, goal.X.z;
    R.col(1) << goal.Y.x, goal.Y.y, goal.Y.z;
    R.col(2) << goal.Z.x, goal.Z.y, goal.Z.z;
    Eigen::Quaterniond q(R);
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();

    group_.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      const auto &pt = plan.trajectory_.joint_trajectory.points.back();
      robot_joint_state j{};
      size_t n = std::min((size_t)robot_joint_state::count,
                          pt.positions.size());
      for (size_t i = 0; i < n; i++)
        j.array[i] = pt.positions[i] * 180.0 / M_PI;
      exchange_moveit_plan.write_begin() = j;
      exchange_moveit_plan.write_end();
      RCLCPP_INFO(get_logger(), "MoveIt plan written with %zu joints", n);
    } else {
      RCLCPP_WARN(get_logger(), "MoveIt planning failed");
    }

    last_goal_ = goal;
  }

  moveit::planning_interface::MoveGroupInterface group_;
  aurora::robot_coord3D last_goal_{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LunaticMoveIt>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
