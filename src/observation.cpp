#include <math.h>
#include "observation.h"
#include "util.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "state_validity.h"


// Sets the start and goal
void ObservationHeuristic::SetProblem(const Eigen::VectorXd& start_in,
                                      const Eigen::VectorXd& goal_in) {
    start_ = start_in;
    goal_ = goal_in;
}


// Computes the uniform heuristic for a configuration and macro-action
double UniformHeuristic::Compute(const Eigen::VectorXd& q, const MacroAction& action) {
    return 1.0;
}


BlindGreedyHeuristic::BlindGreedyHeuristic(double power_in) : power_(power_in) {}


// Computes the blind greedy heuristic for a configuration and macro-action
double BlindGreedyHeuristic::Compute(const Eigen::VectorXd& q, const MacroAction& action) {
    Eigen::VectorXd new_q = q + (action.path.back() - action.path.front());
    // Use inverse distance
    double dist = L2(new_q, goal_);
    return 1.0 / pow(dist, power_);
}


AStarHeuristic::AStarHeuristic(double power_in, double weight_in) :
    power_(power_in), weight_(weight_in) {}


// Computes the A* heuristic for a configuration and macro-action
double AStarHeuristic::Compute(const Eigen::VectorXd& q, const MacroAction& action) {
    Eigen::VectorXd new_q = q + (action.path.back() - action.path.front());
    // Use inverse distance
    double dist = weight_*L2(new_q, goal_) + L1(q, new_q);
    return 1.0 / pow(dist, power_);
}


// TODO(tweiheng): pass in pointer to planning scene
FreedomHeuristic::FreedomHeuristic(robot_state::RobotStatePtr kinematic_state_in, planning_scene::PlanningScenePtr planning_scene_in, const moveit::core::JointModelGroup* jmg, 
           StateValidityChecker* const state_validity_checker_in, collision_detection::AllowedCollisionMatrix acm_in, double freedom_power_in, double goal_power_in) :
    kinematic_state_(kinematic_state_in), planning_scene_(planning_scene_in), joint_model_group(jmg), state_validity_checker_(state_validity_checker_in), acm_(acm_in), freedom_power_(freedom_power_in), goal_power_(goal_power_in) {}


// Computes the freedom heuristic for a configuration and macro-action
double FreedomHeuristic::Compute(const Eigen::VectorXd& q, const MacroAction& action) {
    // double penalty = 0;
    // for(const auto& p : action.path) {
    //     if(!state_validity_checker_->CheckValidity(p)) {
    //         penalty = 0.1;
    //         break;
    //     }
    // }

    Eigen::VectorXd new_q = q + (action.path.back() - action.path.front());
    kinematic_state_->setJointGroupPositions(joint_model_group, new_q);

    double obs_dist = planning_scene_->distanceToCollision(*kinematic_state_, acm_); // TODO(tweiheng): get obstacle distance with the planning scene pointer
    double dist = L2(new_q, goal_);
    // double dist = 1;
    // return pow(obs_dist, freedom_power_) / pow(dist, goal_power_);
    return  pow(dist, goal_power_) / pow(obs_dist, freedom_power_);

}