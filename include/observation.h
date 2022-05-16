#ifndef OBSERVATION_H
#define OBSERVATION_H

#include <Eigen/Dense>
#include "util.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

#include "state_validity.h"

class ObservationHeuristic {
public:
    // Sets the start and goal
    void SetProblem(const Eigen::VectorXd& start_in, const Eigen::VectorXd& goal_in);

    // Computes the observation heuristic for a configuration and macro-action
    virtual double Compute(const Eigen::VectorXd& q, const MacroAction& action) = 0;

protected:
    // Planning context
    Eigen::VectorXd start_;
    Eigen::VectorXd goal_;
};


class UniformHeuristic : public ObservationHeuristic {
public:
    // Computes the uniform heuristic for a configuration and macro-action
    double Compute(const Eigen::VectorXd& q, const MacroAction& action) override;
};


class BlindGreedyHeuristic : public ObservationHeuristic {
public:
    BlindGreedyHeuristic(double power_in=2.0);

    // Computes the blind greedy heuristic for a configuration and macro-action
    double Compute(const Eigen::VectorXd& q, const MacroAction& action) override;

private:
    double power_;
};


class AStarHeuristic : public ObservationHeuristic {
public:
    AStarHeuristic(double power_in=2.0, double weight_in=1.0);

    // Computes the A* heuristic for a configuration and macro-action
    double Compute(const Eigen::VectorXd& q, const MacroAction& action) override;

private:
    double power_;
    double weight_;
};


class FreedomHeuristic : public ObservationHeuristic {
public:
    // TODO(tweiheng): pass in a pointer to the planning scene
    FreedomHeuristic(robot_state::RobotStatePtr kinematic_state_in, planning_scene::PlanningScenePtr planning_scene_in, const moveit::core::JointModelGroup* jmg, 
           StateValidityChecker* const state_validity_checker_in, collision_detection::AllowedCollisionMatrix acm_in, double freedom_power_in=1.0, double goal_power_in=1.0);

    // Computes the freedom heuristic for a configuration and macro-action
    double Compute(const Eigen::VectorXd& q, const MacroAction& action) override;

private:
    // TODO(tweiheng): pointer to planning scene
    double freedom_power_;
    double goal_power_;
    robot_state::RobotStatePtr kinematic_state_;
    StateValidityChecker* const state_validity_checker_;
    collision_detection::AllowedCollisionMatrix acm_;
    planning_scene::PlanningScenePtr planning_scene_;
    const moveit::core::JointModelGroup* joint_model_group;
};
#endif // OBSERVATION_H