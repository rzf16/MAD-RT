#include <math.h>
#include "observation.h"
#include "util.h"


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
FreedomHeuristic::FreedomHeuristic(double freedom_power_in, double goal_power_in) :
    freedom_power_(freedom_power_in), goal_power_(goal_power_in) {}


// Computes the freedom heuristic for a configuration and macro-action
double FreedomHeuristic::Compute(const Eigen::VectorXd& q, const MacroAction& action) {
    Eigen::VectorXd new_q = q + (action.path.back() - action.path.front());
    double obs_dist = 1.0; // TODO(tweiheng): get obstacle distance with the planning scene pointer
    double dist = L2(new_q, goal_);
    return pow(obs_dist, freedom_power_) / pow(dist, goal_power_);
}