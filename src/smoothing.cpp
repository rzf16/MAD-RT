#include <math.h>
#include <Eigen/Dense>
#include "smoothing.h"
#include "util.h"

// TODO(tweiheng): pass in pointers to planning scene & joint model group
ShortcutSmoother::ShortcutSmoother(StateValidityChecker* const state_validity_checker_in,
                                   double step_size_in)
                                   : state_validity_checker_(state_validity_checker_in),
                                     step_size_(step_size_in) {}

// Smooth a path for the given number of iterations
void ShortcutSmoother::Smooth(std::vector<Eigen::VectorXd>& path, size_t num_iterations) {
    for(size_t i = 0; i < num_iterations; ++i) {
        std::uniform_int_distribution<int> distribution(0, path.size()-1);
        size_t a = distribution(rng_);
        size_t b = distribution(rng_);
        // Keep sampling until they are different
        while(a == b) {
            a = distribution(rng_);
            b = distribution(rng_);
        }

        if(a < b) StepBetweenStates(path, a, b);
        else StepBetweenStates(path, b, a);
    }
}

// Step between states a < b
void ShortcutSmoother::StepBetweenStates(std::vector<Eigen::VectorXd>& path, size_t a, size_t b) {
    double dist = L2(path[a], path[b]);
    Eigen::VectorXd step = ((path[b] - path[a]) / dist) * step_size_;
    size_t num_steps = ceil(dist / step_size_);

    std::vector<Eigen::VectorXd> shortcut;
    Eigen::VectorXd current = path[a];
    for(size_t i = 0; i < num_steps; ++i) {
        current = (i == num_steps-1) ? path[b] : current + step;
        if(!state_validity_checker_->CheckValidity(current)) return;
        shortcut.push_back(current);
    }

    if(!shortcut.empty()) {
        path.erase(path.begin()+a+1, path.begin()+b);
        path.insert(path.begin()+a+1, shortcut.begin(), shortcut.end()-1);
    }
}