#include "state_validity.h"
#include <Eigen/Dense>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// TODO(tweiheng): init with pointers to planning scene & joint model group
StateValidityChecker::StateValidityChecker(planning_scene::PlanningScene *ps) {
    planning_scene = ps;
}

// Checks if a configuration is valid
bool StateValidityChecker::CheckValidity(const Eigen::VectorXd& q) {
    // TODO(tweiheng): check validity (collisions + joint limits) using MoveIt
    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    current_state.setJointGroupPositions(joint_model_group, q);

    planning_scene.checkCollision(collision_request, collision_result, current_state);
    return collision_result.collision;
}