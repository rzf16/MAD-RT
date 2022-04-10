#include <iostream>
#include "hmm.h"

HMM::HMM(size_t num_macro_actions) {
    // Initialize uniformly
    transition_counts_ = Eigen::MatrixXd::Constant(num_macro_actions, num_macro_actions, 1);
}

// TODO(rzfeng): add some checks to make sure the initial transition matrix makes sense
HMM::HMM(size_t num_macro_actions, const Eigen::MatrixXd& init_transition_counts) :
    transition_counts_(init_transition_counts) {}

// Updates the transition probabilities given a new macro-action sequence
void HMM::UpdateTransitions(const std::vector<size_t>& sequence) {
    for(size_t i = 0; i < sequence.size()-1; ++i) {
        // Make sure indices are in-bounds
        if(sequence[i] > transition_counts_.rows() || sequence[i+1] > transition_counts_.rows()) {
            std::cout << "[HMM] Encountered out-of-bounds macro-action type during update" << '\n';
            continue;
        }

        transition_counts_(sequence[i], sequence[i+1]) += 1;
    }
    std::cout << transition_counts_ << '\n';
}

// Samples a new macro-action given the previous macro-action and the observation heuristic
size_t HMM::SampleMacroAction(size_t prev, const Eigen::VectorXd& observation_heuristic) {
    // Make sure index in in-bounds
    if(prev > transition_counts_.rows()) {
        std::cout << "[HMM] Encountered out-of-bounds macro-action type during sampling" << '\n';
        return 0;
    }

    // TODO(rzfeng): add a check for observation heuristic shape

    Eigen::VectorXd transition_probs = transition_counts_.row(prev);
    transition_probs /= transition_probs.sum();

    Eigen::VectorXd observation_probs = observation_heuristic / observation_heuristic.sum();

    Eigen::VectorXd weights = transition_probs.cwiseProduct(observation_probs);
    std::vector<double> std_weights(weights.data(), weights.data() + weights.size());

    std::discrete_distribution<size_t> distribution(std_weights.begin(), std_weights.end());
    return distribution(rng);
}