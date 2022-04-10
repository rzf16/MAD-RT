#ifndef STATE_VALIDITY_H
#define STATE_VALIDITY_H

#include <Eigen/Dense>

class StateValidityChecker {
public:
    StateValidityChecker(); // TODO(tweiheng): init with pointers to planning scene & joint model group

    // Checks if a configuration is valid
    bool CheckValidity(const Eigen::VectorXd& q);

private:
    // TODO(tweiheng): add pointers to planning scene & joint model group
};

#endif // STATE_VALIDITY_H