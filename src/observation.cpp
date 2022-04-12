#include <math.h>
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
                                           const Eigen::VectorXd& goal_in,
                                           double power_in) :
    ObservationHeuristic(start_in, goal_in), power_(power_in) {}



// Computes the blind greedy heuristic for a configuration and macro-action
double BlindGreedyHeuristic::Compute(const Eigen::VectorXd& q, const MacroAction& action) {
    Eigen::VectorXd new_q = q + (action.path.back() - action.path.front());
    // Use inverse distance
    double dist = L2(new_q, goal_);
    return 1.0 / pow(dist, power_);
}


AStarHeuristic::AStarHeuristic(const Eigen::VectorXd& start_in,
                               const Eigen::VectorXd& goal_in,
                               double power_in, double weight_in) :
    ObservationHeuristic(start_in, goal_in), power_(power_in), weight_(weight_in) {}



// Computes the A* heuristic for a configuration and macro-action
double AStarHeuristic::Compute(const Eigen::VectorXd& q, const MacroAction& action) {
    Eigen::VectorXd new_q = q + (action.path.back() - action.path.front());
    // Use inverse distance
    double dist = weight_*L2(new_q, goal_) + L1(q, new_q);
    return 1.0 / pow(dist, power_);
}