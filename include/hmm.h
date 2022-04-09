#ifndef HMM_H
#define HMM_H

#include <vector>
#include <random>
#include "util.h"
#include <Eigen/Dense>

class HMM {
public:
    HMM(size_t num_macro_actions);

    HMM(size_t num_macro_actions, const Eigen::MatrixXd& init_transition_counts);

    // Updates the transition probabilities given a new macro-action sequence
    void UpdateTransitions(const std::vector<size_t>& sequence);

    // Samples a new macro-action given the previous macro-action and the observation heuristic
    size_t SampleMacroAction(size_t prev, const Eigen::VectorXd& observation_heuristic);

private:
    Eigen::MatrixXd transition_counts_;
    std::default_random_engine rng;
};

#endif // HMM_H