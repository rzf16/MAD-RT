#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "madrt");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "panda_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  
  
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.id = "box1";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 1.0;
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.25;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  
  // planning_scene_interface.addCollisionObjects(collision_objects);

  // Collision check per point

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);


  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  std::vector<double> check_pt{-0.191, -0.174, 0.0697, -1.849, 0.348, 1.657, 0.662};

  // collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  current_state.setJointGroupPositions(joint_model_group, check_pt);

  planning_scene.checkCollision(collision_request, collision_result, current_state);
  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " collision");



  // Adding traj to vectors

  // std::vector<Eigen::VectorXd> traj;
  // moveit_msgs::RobotTrajectory madrt_traj;
  // // madrt_traj.joint_trajectory.points = trajectory_msgs::JointTrajectoryPoint[traj.size()];
  
  // int count = 0;
  // for(auto &config: traj){
  //   trajectory_msgs::JointTrajectoryPoint pt;
  //   for(int i=0; i < config.size(); i++){
  //     pt.positions.push_back(config(i));
  //   }
  //   madrt_traj.joint_trajectory.points.push_back(pt);
  // }

  // move_group_interface.execute(madrt_traj);

  // 

  ros::shutdown();
  return 0;
}

double configDistance(Eigen::VectorXd goal, Eigen::VectorXd current)
{
  Eigen::VectorXd dist = goal - current;
  return dist.norm();
}

double eeDistance(std::vector<VectorXd> obstacles, Eigen::VectorXd current, )