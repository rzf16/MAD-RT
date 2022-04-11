#include "observation.h"
#include "util.h"

ObservationHeuristic::ObservationHeuristic(const Eigen::VectorXd& start_in,
                                           const Eigen::VectorXd& goal_in) :
    start_(start_in), goal_(goal_in) {}


UniformHeuristic::UniformHeuristic(const Eigen::VectorXd& start_in,
                                   const Eigen::VectorXd& goal_in) :
    ObservationHeuristic(start_in, goal_in) {}


// Computes the uniform heuristic for a configuration and macro-action
double UniformHeuristic::Compute(const Eigen::VectorXd& q, const MacroAction& action) {
    return 1.0;
}


BlindGreedyHeuristic::BlindGreedyHeuristic(const Eigen::VectorXd& start_in,
                                           const Eigen::VectorXd& goal_in) :
    ObservationHeuristic(start_in, goal_in) {}



// Computes the blind greedy heuristic for a configuration and macro-action
double BlindGreedyHeuristic::Compute(const Eigen::VectorXd& q, const MacroAction& action) {
    Eigen::VectorXd new_q = q + (action.path.back() - action.path.front());
    // Use inverse squared distance
    double dist = CalcDistance(new_q, goal_);
    return 1.0 / (dist * dist);
}