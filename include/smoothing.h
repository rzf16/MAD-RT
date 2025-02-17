#ifndef SMOOTHING_H
#define SMOOTHING_H

#include <vector>
#include <random>
#include <Eigen/Dense>
#include "state_validity.h"

class ShortcutSmoother {
public:
    ShortcutSmoother(StateValidityChecker* const state_validity_checker_in,
                     double step_size_in=0.1);

    // Smooth a path for the given number of iterations
    void Smooth(std::vector<Eigen::VectorXd>& path, size_t num_iterations=200);

private:
    // Step between states a < b
    void StepBetweenStates(std::vector<Eigen::VectorXd>& path, size_t a, size_t b);

    double step_size_;
    StateValidityChecker* const state_validity_checker_;
    std::default_random_engine rng_;
};

#endif // SMOOTHING_H