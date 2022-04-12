#ifndef OBSERVATION_H
#define OBSERVATION_H

#include <Eigen/Dense>
#include "util.h"


class ObservationHeuristic {
public:
    ObservationHeuristic(const Eigen::VectorXd& start_in, const Eigen::VectorXd& goal_in);

    // Computes the observation heuristic for a configuration and macro-action
    virtual double Compute(const Eigen::VectorXd& q, const MacroAction& action) = 0;

protected:
    // Planning context
    Eigen::VectorXd start_;
    Eigen::VectorXd goal_;
};


class UniformHeuristic : ObservationHeuristic {
public:
    UniformHeuristic(const Eigen::VectorXd& start_in, const Eigen::VectorXd& goal_in);

    // Computes the uniform heuristic for a configuration and macro-action
    double Compute(const Eigen::VectorXd& q, const MacroAction& action) override;
};


class BlindGreedyHeuristic : ObservationHeuristic {
public:
    BlindGreedyHeuristic(const Eigen::VectorXd& start_in, const Eigen::VectorXd& goal_in,
                         double power_in=2.0);

    // Computes the blind greedy heuristic for a configuration and macro-action
    double Compute(const Eigen::VectorXd& q, const MacroAction& action) override;

private:
    double power_;
};


class AStarHeuristic : ObservationHeuristic {
public:
    AStarHeuristic(const Eigen::VectorXd& start_in, const Eigen::VectorXd& goal_in,
                   double power_in=2.0, double weight_in=1.0);

    // Computes the blind greedy heuristic for a configuration and macro-action
    double Compute(const Eigen::VectorXd& q, const MacroAction& action) override;

private:
    double power_;
    double weight_;
};

#endif // OBSERVATION_H