#ifndef STATE_VALIDITY_H
#define STATE_VALIDITY_H

#include <Eigen/Dense>

class StateValidityChecker {
public:
    StateValidityChecker(planning_scene::PlanningScene *planning_scene); // TODO(tweiheng): init with pointers to planning scene & joint model group

    // Checks if a configuration is valid
    bool CheckValidity(const Eigen::VectorXd& q);

private:
    // TODO(tweiheng): add pointers to planning scene & joint model group
    planning_scene::PlanningScene *planning_scene;
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
};

#endif // STATE_VALIDITY_H