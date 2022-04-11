#ifndef STATE_VALIDITY_H
#define STATE_VALIDITY_H

#include <Eigen/Dense>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

class StateValidityChecker {
public:
    // StateValidityChecker(std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> ps, const moveit::core::JointModelGroup* jmg); // TODO(tweiheng): init with pointers to planning scene & joint model group
    StateValidityChecker(planning_scene::PlanningScenePtr ps, const moveit::core::JointModelGroup* jmg);
    // Checks if a configuration is valid
    bool CheckValidity(const Eigen::VectorXd& q);

private:
    // TODO(tweiheng): add pointers to planning scene & joint model group
    // std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm;
    const moveit::core::JointModelGroup* joint_model_group;
    planning_scene::PlanningScenePtr planning_scene;
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
};

#endif // STATE_VALIDITY_H