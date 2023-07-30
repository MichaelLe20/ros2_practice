#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move");

  // 创建一个线程
  // We spin up a SingleThreadedExecutor for the current state monitor
  // to get information about the robot's state.
  // https://docs.ros2.org/crystal/api/rclcpp/classrclcpp_1_1executors_1_1SingleThreadedExecutor.html
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); // 添加 node 执行其回调函数（可以多个）
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface =
      MoveGroupInterface(node, "ur5_arm"); // planning group

  // Construct and initialize MoveItVisualTools
  // Parameters:
  // - node
  // - base frame (as common base for all visualization markers)
  //   NOTE: the frame name is link name (usually be /world)
  // - the rostopic to publish markers
  //   NOTE: static const std::string RVIZ_MARKER_TOPIC = "/rviz_visual_tools";
  // - robot model
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface
          .getRobotModel()}; // 初始化
                             // moveit_visual_tools（花括号{}进行列表初始化）
  moveit_visual_tools.deleteAllMarkers();  // clear markers in RViz
  moveit_visual_tools.loadRemoteControl(); // Remote control is a simple plugin
                                           // that lets us have a button in RViz
                                           // to interact with our program.

  // Create closures for updating the text in rviz
  // 创建匿名函数快速进行相关操作 (绘制函数)
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0; // Place text 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  }; // post a given text 1m above the base link
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  }; // pause and wait for user condirmation to continue
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
       jmg = move_group_interface.getRobotModel()->getJointModelGroup(
           "ur5_arm")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      }; // draw a trajectory line

  // 正式操作
  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  // NOTE：尽管目标已经达到，但是重复 plan 会生成不一样的轨迹，不知道为什么

  // Execute the plan
  if (success) {
    draw_trajectory_tool_path(
        plan.trajectory_); // ❗版本不同，可能是 plan.trajectory
    moveit_visual_tools.trigger(); // 不知为何我使用时没有显示轨迹
    prompt("Press 'next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown(); // This will cause the spin function in the thread to
                      // return
  spinner.join();     // Join the thread before exiting
                      // 阻塞当前线程直到 spinner 线程返回
  return 0;
}