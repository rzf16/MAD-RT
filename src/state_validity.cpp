#include "state_validity.h"
#include <Eigen/Dense>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>
// TODO(tweiheng): init with pointers to planning scene & joint model group
StateValidityChecker::StateValidityChecker(planning_scene::PlanningScenePtr ps, const moveit::core::JointModelGroup* jmg) {
    planning_scene = ps;
    joint_model_group = jmg;

}

// Checks if a configuration is valid
bool StateValidityChecker::CheckValidity(const Eigen::VectorXd& q) {

    // psm->requestPlanningSceneState("/get_planning_scene");
    // planning_scene::PlanningScenePtr planning_scene = psm->getPlanningScene();

    moveit::core::RobotState& current_state = planning_scene->getCurrentStateNonConst();
    current_state.setJointGroupPositions(joint_model_group, q);
    collision_result.clear();
    planning_scene->checkCollision(collision_request, collision_result, current_state);
    return !collision_result.collision;// && current_state.satisfiesBounds();
}