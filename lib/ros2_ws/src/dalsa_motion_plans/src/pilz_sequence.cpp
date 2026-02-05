#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>

using moveit::planning_interface::MoveGroupInterface;
using moveit_msgs::action::MoveGroupSequence;
using GoalHandleMoveGroupSequence = rclcpp_action::ClientGoalHandle<MoveGroupSequence>;

using GetMotionSequence = moveit_msgs::srv::GetMotionSequence;
using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;

class TrajectorySequence
{
private:
  std::string planning_group_;
  rclcpp::Client<GetMotionSequence>::SharedPtr seq_service_client_;
  rclcpp_action::Client<MoveGroupSequence>::SharedPtr seq_action_client_;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<MoveGroupInterface> mgi_;

  std::vector<moveit_msgs::msg::MotionSequenceItem> items_;

  void exec_thread(void)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    executor.spin();
  }

public:
  TrajectorySequence(std::string planning_group)
  {
    planning_group_ = planning_group;
  }

  ~TrajectorySequence() {};

  int init(void)
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    node_ = rclcpp::Node::make_shared("pilz_sequence_node", node_options);

    // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
    std::thread t([this]
                  { exec_thread(); });
    t.detach();

    mgi_ = std::make_shared<MoveGroupInterface>(node_, planning_group_);

    // [-------------------- Create Service and Action Clients --------------------]

    // MoveGroupSequence service client
    seq_service_client_ = node_->create_client<GetMotionSequence>("/plan_sequence_path");

    // Verify that the action server is up and running
    while (!seq_service_client_->wait_for_service(std::chrono::seconds(10)))
    {
      RCLCPP_WARN(node_->get_logger(), "Waiting for service /plan_sequence_path to be available...");
    }

    // MoveGroupSequence action client
    seq_action_client_ = rclcpp_action::create_client<MoveGroupSequence>(node_, "/sequence_move_group");

    // Verify that the action server is up and running
    if (!seq_action_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Error waiting for action server /sequence_move_group");
      return -1;
    }

    return 0;
  }

  std::shared_ptr<MoveGroupInterface> getMoveGroupInterface(void)
  {
    return mgi_;
  }

  rclcpp::Node::SharedPtr getNode(void)
  {
    return node_;
  }

  bool addSegment(moveit_msgs::msg::MotionSequenceItem segment)
  {
    items_.push_back(segment);
    return true;
  }

  bool execute(void)
  {
    // Create a MotionSequenceRequest
    moveit_msgs::msg::MotionSequenceRequest sequence_request;
    sequence_request.items = items_;

    // Create action goal
    auto goal_msg = MoveGroupSequence::Goal();
    goal_msg.request = sequence_request;

    // Planning options
    goal_msg.planning_options.planning_scene_diff.is_diff = true;
    goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;
    // goal_msg.planning_options.plan_only = true; // Uncomment to only plan the trajectory

    // auto const draw_trajectory_tool_path =
    //     [&moveit_visual_tools,
    //      robot = getMoveGroupInterface()->getRobotModel()](auto const &trajectories)
    // {
    //   auto jmg = robot->getJointModelGroup(PLANNING_GROUP);
    //   for (const auto &trajectory : trajectories)
    //   {
    //     moveit_visual_tools.publishTrajectoryLine(trajectory, robot->getLinkModel(PLANNING_FRAME), jmg);
    //   }
    // };

    // Goal response callback
    auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleMoveGroupSequence> goal_handle)
    {
      try
      {
        if (!goal_handle)
        {
          RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        }
        else
        {
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(node_->get_logger(), "Exception while waiting for goal response: %s", e.what());
      }
    };

    // Result callback
    send_goal_options.result_callback = [this](const GoalHandleMoveGroupSequence::WrappedResult &result)
    {
      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was aborted. Status: %d", result.result->response.error_code.val);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
        break;
      }
      RCLCPP_INFO(node_->get_logger(), "Result received");
    };

    // Send the action goal
    auto goal_handle_future = seq_action_client_->async_send_goal(goal_msg, send_goal_options);

    // Get result
    auto action_result_future = seq_action_client_->async_get_result(goal_handle_future.get());

    // Wait for the result
    std::future_status action_status;
    do
    {
      switch (action_status = action_result_future.wait_for(std::chrono::seconds(1)); action_status)
      {
      case std::future_status::deferred:
        RCLCPP_ERROR(node_->get_logger(), "Deferred");
        break;
      case std::future_status::timeout:
        RCLCPP_INFO(node_->get_logger(), "Executing trajectory...");
        break;
      case std::future_status::ready:
        RCLCPP_INFO(node_->get_logger(), "Action ready!");
        break;
      }
    } while (action_status != std::future_status::ready);

    if (action_result_future.valid())
    {
      auto result = action_result_future.get();
      RCLCPP_INFO(node_->get_logger(), "Action completed. Result: %d", static_cast<int>(result.code));
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Action couldn't be completed.");
      return false;
    }

    items_.clear();
    return true;
  }

  /**
   * The methods are very redundant and hard coded. Things that come to mind:
   - refactor common functions
   - set names of pipeline_id in
   - dont need to be member functions
   */

  moveit_msgs::msg::MotionSequenceItem moveJ(const std::string &link_name, const geometry_msgs::msg::PoseStamped &pose, const float blend_radius = 0.0, const float speed_scaling = 1.0)
  {
    moveit_msgs::msg::MotionSequenceItem segment;

    // Set pose blend radius
    segment.blend_radius = blend_radius;

    // MotionSequenceItem configuration
    segment.req.group_name = planning_group_;

    segment.req.pipeline_id = "ompl";
    segment.req.allowed_planning_time = 5.0;
    segment.req.max_velocity_scaling_factor = speed_scaling;
    segment.req.max_acceleration_scaling_factor = speed_scaling;

    segment.req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(link_name, pose));

    return segment;
  }

  moveit_msgs::msg::MotionSequenceItem moveJ(const std::vector<moveit_msgs::msg::JointConstraint> &joint_values, const float blend_radius = 0.0, const float speed_scaling = 1.0)
  {
    moveit_msgs::msg::MotionSequenceItem segment;

    // Set pose blend radius
    segment.blend_radius = blend_radius;

    // MotionSequenceItem configuration
    segment.req.group_name = planning_group_;

    segment.req.pipeline_id = "ompl";
    segment.req.allowed_planning_time = 5.0;
    segment.req.max_velocity_scaling_factor = speed_scaling;
    segment.req.max_acceleration_scaling_factor = speed_scaling;

    moveit_msgs::msg::Constraints goal_constraint;
    goal_constraint.joint_constraints = joint_values;
    segment.req.goal_constraints.push_back(goal_constraint);

    return segment;
  }

  moveit_msgs::msg::MotionSequenceItem moveL(const std::string &link_name, const geometry_msgs::msg::PoseStamped &pose, const float blend_radius = 0.0, const float speed_scaling = 1.0)
  {
    moveit_msgs::msg::MotionSequenceItem segment;

    // Set pose blend radius
    segment.blend_radius = blend_radius;

    // MotionSequenceItem configuration
    segment.req.group_name = planning_group_;

    segment.req.planner_id = "LIN";
    segment.req.pipeline_id = "pilz_industrial_motion_planner";
    segment.req.allowed_planning_time = 5.0;
    segment.req.max_velocity_scaling_factor = speed_scaling;
    segment.req.max_acceleration_scaling_factor = speed_scaling;

    segment.req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(link_name, pose));

    return segment;
  }

  moveit_msgs::msg::MotionSequenceItem moveL(const std::vector<moveit_msgs::msg::JointConstraint> &joint_values, const float blend_radius = 0.0, const float speed_scaling = 1.0)
  {
    moveit_msgs::msg::MotionSequenceItem segment;

    // Set pose blend radius
    segment.blend_radius = blend_radius;

    // MotionSequenceItem configuration
    segment.req.group_name = planning_group_;

    segment.req.planner_id = "LIN";
    segment.req.pipeline_id = "pilz_industrial_motion_planner";
    segment.req.allowed_planning_time = 5.0;
    segment.req.max_velocity_scaling_factor = speed_scaling;
    segment.req.max_acceleration_scaling_factor = speed_scaling;

    moveit_msgs::msg::Constraints goal_constraint;
    goal_constraint.joint_constraints = joint_values;
    segment.req.goal_constraints.push_back(goal_constraint);

    return segment;
  }
};

// Create a ROS logger
auto const LOGGER = rclcpp::get_logger("pilz_sequence_node");

int main(int argc, char **argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  TrajectorySequence SeqPlanner("arm");

  if (SeqPlanner.init() < 0)
  {
    RCLCPP_ERROR(LOGGER, "Failed to init.");
    return -1;
  }

  // Planning group
  static const std::string PLANNING_GROUP = "arm";
  static const std::string PLANNING_FRAME = "tool";

  // Create the MoveIt MoveGroup Interface
  // In this case, this is just necessary for the visual interface
  // using moveit::planning_interface::MoveGroupInterface;
  // auto move_group_interface = MoveGroupInterface(node_, PLANNING_GROUP);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{SeqPlanner.getNode(), "base_link", "motion_plan",
                                                                    SeqPlanner.getMoveGroupInterface()->getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  // moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  geometry_msgs::msg::PoseStamped pose_1;
  pose_1.header.frame_id = "world";
  pose_1.pose.position.x = 0.5;
  pose_1.pose.position.y = -0.2;
  pose_1.pose.position.z = 0.2;
  pose_1.pose.orientation.z = 0.0;
  pose_1.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped pose_2;
  pose_2.header.frame_id = "world";
  pose_2.pose.position.x = 0.5;
  pose_2.pose.position.y = 0.2;
  pose_2.pose.position.z = 0.2;
  pose_2.pose.orientation.z = 0.0;
  pose_2.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped pose_3;
  pose_3.header.frame_id = "world";
  pose_3.pose.position.x = 0.4;
  pose_3.pose.position.y = 0.2;
  pose_3.pose.position.z = 0.2;
  pose_3.pose.orientation.z = 0.0;
  pose_3.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped pose_4;
  pose_4.header.frame_id = "world";
  pose_4.pose.position.x = 0.4;
  pose_4.pose.position.y = -0.2;
  pose_4.pose.position.z = 0.2;
  pose_4.pose.orientation.z = 0.0;
  pose_4.pose.orientation.w = 1.0;

  SeqPlanner.addSegment(SeqPlanner.moveJ(PLANNING_FRAME, pose_1, 0.01, 0.5));
  SeqPlanner.addSegment(SeqPlanner.moveJ(PLANNING_FRAME, pose_2, 0.01, 0.5));
  SeqPlanner.addSegment(SeqPlanner.moveJ(PLANNING_FRAME, pose_3, 0.01, 0.5));
  SeqPlanner.addSegment(SeqPlanner.moveJ(PLANNING_FRAME, pose_4, 0.0, 0.5));
  SeqPlanner.execute();

  rclcpp::shutdown();
  return 0;
  /*



  // [ --------------------------------------------------------------- ]
  // [ ----------------------- Motion Sequence ----------------------- ]
  // [ --------------------------------------------------------------- ]

  // ----- Motion Sequence Item 1
  // Create a MotionSequenceItem
  moveit_msgs::msg::MotionSequenceItem item1;

  // Set pose blend radius
  item1.blend_radius = 0.1;

  // MotionSequenceItem configuration
  item1.req.group_name = PLANNING_GROUP;

  // item1.req.planner_id = "LIN";
  item1.req.pipeline_id = "ompl";
  // item1.req.pipeline_id = "pilz_industrial_motion_planner";
  item1.req.allowed_planning_time = 5.0;
  item1.req.max_velocity_scaling_factor = 0.5;
  item1.req.max_acceleration_scaling_factor = 0.5;

  moveit_msgs::msg::Constraints constraints_item1;
  // moveit_msgs::msg::PositionConstraint pos_constraint_item1;

  // // Set a constraint pose
  auto pos_target_pose_item1 = []
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position.x = 0.4;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.2;
    // msg.pose.orientation.x = 0.000001;  // X and Y components must be zero (or omitted)
    // msg.pose.orientation.y = 0.000001;  // X and Y components must be zero
    // msg.pose.orientation.z = 1.0;
    msg.pose.orientation.w = 1.0;
    return msg;
  }();

  item1.req.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(PLANNING_FRAME, pos_target_pose_item1));

  std::vector<moveit_msgs::msg::JointConstraint> joint_constraints_item1;

  // ----- Motion Sequence Item 2
  // Create a MotionSequenceItem
  moveit_msgs::msg::MotionSequenceItem item2;

  // Set pose blend radius
  // For the last pose, it must be 0!
  item2.blend_radius = 0.0;

  // MotionSequenceItem configuration
  item2.req.group_name = PLANNING_GROUP;
  item2.req.planner_id = "LIN";
  // item2.req.planner_id = "TRRTkConfigDefault";
  // item2.req.pipeline_id = "ompl";
  item2.req.pipeline_id = "pilz_industrial_motion_planner";
  item2.req.allowed_planning_time = 5.0;
  item2.req.max_velocity_scaling_factor = 0.5;
  item2.req.max_acceleration_scaling_factor = 0.5;

  // Set a constraint pose
  auto target_pose_item2 = []
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position.x = 0.4;
    msg.pose.position.y = -0.3;
    msg.pose.position.z = 0.2;
    // msg.pose.orientation.x = 0.0;
    // msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    return msg;
  }();
  item2.req.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(PLANNING_FRAME, target_pose_item2, 0.1, 1.0));

  // [ --------------------------------------------------------------- ]
  // [ ------------------ MoveGroupSequence Service ------------------ ]
  // [ --------------------------------------------------------------- ]
  // The trajectory is returned but not executed

  // Create request
  // auto seq_service_request = std::make_shared<GetMotionSequence::Request>();
  // seq_service_request->request.items.push_back(item1);
  // seq_service_request->request.items.push_back(item2);

  // // Call the service and process the result
  // auto seq_service_future = SeqPlanner.seq_service_client_->async_send_request(seq_service_request); // TODO use getter

  // // Function to draw the trajectory
  // auto const draw_trajectory_tool_path =
  //     [&moveit_visual_tools,
  //      robot = SeqPlanner.getMoveGroupInterface()->getRobotModel()](auto const &trajectories)
  // {
  //   auto jmg = robot->getJointModelGroup(PLANNING_GROUP);
  //   for (const auto &trajectory : trajectories)
  //   {
  //     moveit_visual_tools.publishTrajectoryLine(trajectory, robot->getLinkModel(PLANNING_FRAME), jmg);
  //   }
  // };

  // // Wait for the result
  // std::future_status service_status;
  // do
  // {
  //   switch (service_status = seq_service_future.wait_for(std::chrono::seconds(1)); service_status)
  //   {
  //   case std::future_status::deferred:
  //     RCLCPP_ERROR(LOGGER, "Deferred");
  //     break;
  //   case std::future_status::timeout:
  //     RCLCPP_INFO(LOGGER, "Waiting for trajectory plan...");
  //     break;
  //   case std::future_status::ready:
  //     RCLCPP_INFO(LOGGER, "Service ready!");
  //     break;
  //   }
  // } while (service_status != std::future_status::ready);

  // auto seq_service_response = seq_service_future.get();
  // if (seq_service_response->response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  // {
  //   RCLCPP_INFO(LOGGER, "Planning successful");

  //   // Access the planned trajectory
  //   auto trajectory = seq_service_response->response.planned_trajectories;

  //   // Publish blended trajectory
  //   rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =
  //       SeqPlanner.getNode()->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
  //   moveit_msgs::msg::DisplayTrajectory display_trajectory;
  //   display_trajectory.trajectory_start = seq_service_response->response.sequence_start;
  //   display_trajectory.trajectory = trajectory;
  //   display_publisher->publish(display_trajectory);

  //   draw_trajectory_tool_path(trajectory);
  //   moveit_visual_tools.trigger();
  // }
  // else
  // {
  //   RCLCPP_ERROR(LOGGER, "Planning failed with error code: %d", seq_service_response->response.error_code.val);

  //   rclcpp::shutdown();
  //   return 0;
  // }

  // moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // [ --------------------------------------------------------------- ]
  // [ ------------------ MoveGroupSequence Action ------------------- ]
  // [ --------------------------------------------------------------- ]
  // Plans and executes the trajectory

  // // Create a MotionSequenceRequest
  // moveit_msgs::msg::MotionSequenceRequest sequence_request;
  // sequence_request.items.push_back(item1);
  // sequence_request.items.push_back(item2);

  // // Create action goal
  // auto goal_msg = MoveGroupSequence::Goal();
  // goal_msg.request = sequence_request;

  // // Planning options
  // goal_msg.planning_options.planning_scene_diff.is_diff = true;
  // goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;
  // // goal_msg.planning_options.plan_only = true; // Uncomment to only plan the trajectory

  // // Goal response callback
  // auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
  // send_goal_options.goal_response_callback = [](std::shared_ptr<GoalHandleMoveGroupSequence> goal_handle)
  // {
  //   try
  //   {
  //     if (!goal_handle)
  //     {
  //       RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
  //     }
  //     else
  //     {
  //       RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
  //     }
  //   }
  //   catch (const std::exception &e)
  //   {
  //     RCLCPP_ERROR(LOGGER, "Exception while waiting for goal response: %s", e.what());
  //   }
  // };

  // // Result callback
  // send_goal_options.result_callback = [](const GoalHandleMoveGroupSequence::WrappedResult &result)
  // {
  //   switch (result.code)
  //   {
  //   case rclcpp_action::ResultCode::SUCCEEDED:
  //     RCLCPP_INFO(LOGGER, "Goal succeeded");
  //     break;
  //   case rclcpp_action::ResultCode::ABORTED:
  //     RCLCPP_ERROR(LOGGER, "Goal was aborted. Status: %d", result.result->response.error_code.val);
  //     break;
  //   case rclcpp_action::ResultCode::CANCELED:
  //     RCLCPP_ERROR(LOGGER, "Goal was canceled");
  //     break;
  //   default:
  //     RCLCPP_ERROR(LOGGER, "Unknown result code");
  //     break;
  //   }
  //   RCLCPP_INFO(LOGGER, "Result received");
  // };

  // // Send the action goal
  // auto goal_handle_future = SeqPlanner.seq_action_client_->async_send_goal(goal_msg, send_goal_options);

  // // Get result
  // auto action_result_future = SeqPlanner.seq_action_client_->async_get_result(goal_handle_future.get());

  // // Wait for the result
  // std::future_status action_status;
  // do
  // {
  //   switch (action_status = action_result_future.wait_for(std::chrono::seconds(1)); action_status)
  //   {
  //   case std::future_status::deferred:
  //     RCLCPP_ERROR(LOGGER, "Deferred");
  //     break;
  //   case std::future_status::timeout:
  //     RCLCPP_INFO(LOGGER, "Executing trajectory...");
  //     break;
  //   case std::future_status::ready:
  //     RCLCPP_INFO(LOGGER, "Action ready!");
  //     break;
  //   }
  // } while (action_status != std::future_status::ready);

  // if (action_result_future.valid())
  // {
  //   auto result = action_result_future.get();
  //   RCLCPP_INFO(LOGGER, "Action completed. Result: %d", static_cast<int>(result.code));
  // }
  // else
  // {
  //   RCLCPP_ERROR(LOGGER, "Action couldn't be completed.");
  // }



      moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

      // [ --------------------------------------------------------------- ]
      // [ ------------------------- Stop Motion ------------------------- ]
      // [ --------------------------------------------------------------- ]

      // Repeats the motion but after 2 seconds cancels the action
      auto cancel_goal_handle_future = seq_action_client_->async_send_goal(goal_msg, send_goal_options);
      sleep(2);
      auto cancel_action_result_future = seq_action_client_->async_cancel_goal(cancel_goal_handle_future.get());

      // Wait for the result
      std::future_status action_cancel_status;
      do
      {
        switch (action_cancel_status = cancel_action_result_future.wait_for(std::chrono::seconds(1)); action_cancel_status)
        {
        case std::future_status::deferred:
          RCLCPP_ERROR(LOGGER, "Deferred");
          break;
        case std::future_status::timeout:
          RCLCPP_INFO(LOGGER, "Waiting for trajectory stop...");
          break;
        case std::future_status::ready:
          RCLCPP_INFO(LOGGER, "Action cancel!");
          break;
        }
      } while (action_cancel_status != std::future_status::ready);

      if (cancel_action_result_future.valid())
      {
        auto cancel_response = cancel_action_result_future.get();

        if (!cancel_response->return_code)
        {
          RCLCPP_INFO(LOGGER, "The action has been canceled by the action server.");
        }
        else
        {
          RCLCPP_INFO(LOGGER, "Action cancel error. Code: %d.", cancel_response->return_code);
        }
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Action couldn't be canceled.");
      }

      rclcpp::shutdown();
      return 0;
      */
}
