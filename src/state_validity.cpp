#include "state_validity.h"
#include <Eigen/Dense>

// TODO(tweiheng): init with pointers to planning scene & joint model group
StateValidityChecker::StateValidityChecker() {}

// Checks if a configuration is valid
bool StateValidityChecker::CheckValidity(const Eigen::VectorXd& q) {
    // TODO(tweiheng): check validity (collisions + joint limits) using MoveIt
    // return true;
    return q[0] > -0.1 && q[0] < 1.2;
}