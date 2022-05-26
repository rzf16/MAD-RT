#include <iostream>
#include <math.h>
#include <algorithm>
#include <chrono>
#include <random>
#include <limits>
#include "mad_rt.h"
#include "state_validity.h"
#include "observation.h"

MAD_RT::MAD_RT(const std::vector<MacroAction>& macro_actions_in,
               const std::vector<std::string>& names_in,
               StateValidityChecker* const state_validity_checker_in,
               ObservationHeuristic* observation_heuristic_in,
               double goal_eps_in,
               double goal_bias_in,
               double trunc_min_in,
               double trunc_max_in,
               double count_penalty_in,
               const Eigen::VectorXd& malleability_in)
              : macro_actions_(macro_actions_in),
                names_(names_in),
                state_validity_checker_(state_validity_checker_in),
                observation_heuristic_(observation_heuristic_in),
                hmm_(HMM(macro_actions_in.size())),
                goal_eps_(goal_eps_in),
                goal_bias_(goal_bias_in),
                trunc_min_(trunc_min_in),
                trunc_max_(trunc_max_in),
                count_penalty_(count_penalty_in),
                malleability_(malleability_in) {}

MAD_RT::MAD_RT(const std::vector<MacroAction>& macro_actions_in,
               const std::vector<std::string>& names_in,
               StateValidityChecker* const state_validity_checker_in,
               ObservationHeuristic* observation_heuristic_in,
               const Eigen::MatrixXd& init_transition_counts,
               double goal_eps_in,
               double goal_bias_in,
               double trunc_min_in,
               double trunc_max_in,
               double count_penalty_in,
               const Eigen::VectorXd& malleability_in)
              : macro_actions_(macro_actions_in),
                names_(names_in),
                state_validity_checker_(state_validity_checker_in),
                observation_heuristic_(observation_heuristic_in),
                hmm_(HMM(macro_actions_in.size(), init_transition_counts)),
                goal_eps_(goal_eps_in),
                goal_bias_(goal_bias_in),
                trunc_min_(trunc_min_in),
                trunc_max_(trunc_max_in),
                count_penalty_(count_penalty_in),
                malleability_(malleability_in) {}

// Plans a path from start to goal
std::vector<Eigen::VectorXd> MAD_RT::plan(const Eigen::VectorXd& start,
                                          const Eigen::VectorXd& goal,
                                          double max_time) {
    observation_heuristic_->SetProblem(start, goal);
    std::chrono::steady_clock::time_point tick = std::chrono::steady_clock::now();

    nodes_.push_back(MAD_RT_Node(std::vector<Eigen::VectorXd>{start},
                                 std::numeric_limits<size_t>::max(),
                                 std::numeric_limits<size_t>::max()));
    node_weights_.push_back(CalcNodeWeight(nodes_[0]));

    // First try to directly morph all macro-actions to the goal
    for(const MacroAction& action : macro_actions_) {
        auto shear_shift = CalcShearShift(action.path, start, goal);
        auto path = Morph(action.path, shear_shift[0], shear_shift[1]);

        bool valid = true;
        for(const auto& q : path) {
            if(!state_validity_checker_->CheckValidity(q)) {
                valid = false;
                break;
            }
        }

        if(valid) {
            std::chrono::steady_clock::time_point tock = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = tock - tick;
            std::cout << "[MAD_RT] Found a path in " << diff.count() << " seconds!" << '\n';
            std::cout << "[MAD_RT] Macro-Action Sequence: " << names_[action.type] << '\n';
            return path;
        }
    }

    // While time not exhausted:
    //     Sample a node
    //     Get observation heuristic for current node
    //     Get a macro-action from HMM using current action and observation
    //     Choose between expansion and goal
    //     Truncate macro-action
    //     Morph macro-action
    //     Check for collisions and joint limits
    //     Add edge to tree
    //     If goal reached: backtrack
    // Return failure

    double elapsed = 0.0;
    size_t i = 0;
    while(elapsed < max_time) {
        // DEBUG
        // std::cout << "[MAD_RT] Iteration " << i++ << '\n';
        // std::cout << "[MAD_RT] " << nodes_.size() << " nodes in tree" << '\n';

        size_t node_id = SampleNode();
        // DEBUG
        // std::cout << "[MAD_RT] Sampled node " << node_id << '\n';

        Eigen::VectorXd observation_weights(macro_actions_.size());
        for(size_t j = 0; j < macro_actions_.size(); ++j) {
            observation_weights(j) = observation_heuristic_->Compute(nodes_[node_id].action.path.back(),
                                                                     macro_actions_[j]);
        }
        // Normalize the weights
        observation_weights = observation_weights / observation_weights.maxCoeff();

        size_t action_type;
        // Use only observation heuristic if no previous action
        if(nodes_[node_id].parent == std::numeric_limits<size_t>::max()) {
            std::discrete_distribution<size_t> distribution(observation_weights.data(),
                                                            observation_weights.data() + observation_weights.size());
            action_type = distribution(rng_);
        }
        else {
            // action_type = hmm_.SampleMacroAction(nodes_[nodes_[node_id].parent].action.type,
            //                                      observation_weights);
            action_type = hmm_.SampleMacroAction(nodes_[node_id].action.type,
                                                 observation_weights);
        }
        // DEBUG
        // std::cout << "[MAD_RT] Sampled action " << names_[action_type] << '\n';

        std::vector<Eigen::VectorXd> path;
        bool sample_goal = ((double)rand() / (double)RAND_MAX) < goal_bias_;
        if(sample_goal) {
            // DEBUG
            // std::cout << "[MAD_RT] Targeting goal" << '\n';

            auto shear_shift = CalcShearShift(macro_actions_[action_type].path,
                                              nodes_[node_id].action.path.back(),
                                              goal);
            path = Morph(macro_actions_[action_type].path, shear_shift[0], shear_shift[1]);
        }
        else {
            // DEBUG
            // std::cout << "[MAD_RT] Expanding tree" << '\n';

            // Truncate macro-action
            std::uniform_real_distribution<double> distribution(trunc_min_, trunc_max_);
            double trunc = distribution(rng_);
            size_t trunc_size = ceil(macro_actions_[action_type].path.size() * trunc);
            if(trunc_size < 2) trunc_size = 2;

            path = macro_actions_[action_type].path;
            path.resize(trunc_size);

            auto shear_shift = SampleShearShift(path, nodes_[node_id].action.path.back(), trunc);
            path = Morph(path, shear_shift[0], shear_shift[1]);
        }

        bool valid = true;
        // TODO(rzfeng): check with interpolation
        for(const auto& q : path) {
            if(!state_validity_checker_->CheckValidity(q)) {
                valid = false;
                break;
            }
        }

        if(valid) {
            nodes_.push_back(MAD_RT_Node(path, action_type, node_id));
            node_weights_.push_back(1.0);

            if(L2(path.back(), goal) < goal_eps_) {
                std::chrono::steady_clock::time_point tock = std::chrono::steady_clock::now();
                std::chrono::duration<double> diff = tock - tick;
                elapsed = diff.count();
                // DEBUG
                std::cout << '\n';
                std::cout << "[MAD_RT] Found a path in " << elapsed << " seconds!" << '\n';

                std::vector<Eigen::VectorXd> motion_plan;
                std::vector<size_t> macro_action_seq;
                size_t current_id = nodes_.size()-1;
                while(nodes_[current_id].parent != std::numeric_limits<size_t>::max()) {
                    // Macro-actions are forward but overall path is backwards, so reverse macro-actions
                    motion_plan.insert(motion_plan.end(), nodes_[current_id].action.path.rbegin(),
                                       nodes_[current_id].action.path.rend());
                    macro_action_seq.push_back(nodes_[current_id].action.type);
                    current_id = nodes_[current_id].parent;
                }
                // We don't have to add the start node, since macro-actions are stored with the end node
                std::reverse(motion_plan.begin(), motion_plan.end());
                std::reverse(macro_action_seq.begin(), macro_action_seq.end());
                hmm_.UpdateTransitions(macro_action_seq);

                // DEBUG
                std::cout << "[MAD_RT] Macro-Action Sequence: ";
                for(size_t i = 0; i < macro_action_seq.size(); ++i) {
                    std::cout << names_[macro_action_seq[i]];
                    if(i < macro_action_seq.size()-1) std::cout << ", ";
                }
                std::cout << '\n';

                return motion_plan;
            }
        }
        else {
            // std::cout << "[MAD_RT] Invalid state!" << '\n';
        }

        std::chrono::steady_clock::time_point tock = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = tock - tick;
        elapsed = diff.count();
        // std::cout << '\n';
    }

    std::cout << "[MAD-RT] Failed to find a path D:" << '\n';
    std::vector<Eigen::VectorXd> empty_path;
    return empty_path;
}

// Morphs a path using shear and shift coefficients
std::vector<Eigen::VectorXd> MAD_RT::Morph(const std::vector<Eigen::VectorXd>& path,
                                           const Eigen::VectorXd& shear,
                                           const Eigen::VectorXd& shift) const {
    std::vector<Eigen::VectorXd> morphed;
    for(size_t i = 0; i < path.size(); ++i) {
        double t = double(i) / double(path.size()-1);
        Eigen::VectorXd q = path[i] + t*shear + shift;
        for(size_t j = 0; j < q.size(); ++j) {
            q(j) = ClampRadians(q(j));
        }
        morphed.push_back(q);
    }

    // Interpolate
    std::vector<Eigen::VectorXd> interpolated;
    for(size_t i = 0; i < morphed.size()-1; ++i) {
        Eigen::VectorXd diff = morphed[i+1] - morphed[i];
        double step_size = 0.2;
        Eigen::VectorXd step = (diff / diff.norm()) * step_size;
        size_t num_steps = ceil(diff.norm() / step_size);

        Eigen::VectorXd current = morphed[i];
        for(size_t j = 0; j < num_steps; ++j) {
            interpolated.push_back(current);
            current = current + step;
        }
    }
    interpolated.push_back(morphed.back());
    return interpolated;
}

// Samples a shear coefficient and computes a shift coefficient
std::array<Eigen::VectorXd, 2> MAD_RT::SampleShearShift(const std::vector<Eigen::VectorXd>& path,
                                                        const Eigen::VectorXd& start,
                                                        double trunc) const {
    Eigen::VectorXd shear = Eigen::VectorXd::Random(start.size());
    shear = shear.cwiseProduct(malleability_) * trunc;

    Eigen::VectorXd shift = start - path[0];

    return std::array<Eigen::VectorXd, 2> {{shear, shift}};
}

// Computes shear and shift coefficients to reach a target
std::array<Eigen::VectorXd, 2> MAD_RT::CalcShearShift(const std::vector<Eigen::VectorXd>& path,
                                                      const Eigen::VectorXd& start,
                                                      const Eigen::VectorXd& target) const {
    Eigen::VectorXd shift = start - path[0];
    Eigen::VectorXd shear = target - (path.back() + shift);

    return std::array<Eigen::VectorXd, 2> {{shear, shift}};
}

// Computes sampling weight for a node
double MAD_RT::CalcNodeWeight(const MAD_RT_Node& node) const {
    return 1.0 / (count_penalty_ * node.sample_count + 1.0);
}

// Samples a node from the tree
size_t MAD_RT::SampleNode() {
    std::discrete_distribution<size_t> distribution(node_weights_.begin(), node_weights_.end());
    size_t node_id = distribution(rng_);
    ++nodes_[node_id].sample_count;
    node_weights_[node_id] = CalcNodeWeight(nodes_[node_id]);
    return node_id;
}
