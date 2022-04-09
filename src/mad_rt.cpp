#include <iostream>
#include <math.h>
#include <algorithm>
#include <chrono>
#include <random>
#include <limits>
#include "mad_rt.h"

MAD_RT::MAD_RT(const std::vector<MacroAction>& macro_actions_in,
               const std::vector<std::string>& names_in,
               double goal_eps_in,
               double goal_bias_in,
               double trunc_min_in,
               double trunc_max_in,
               double count_penalty_in,
               const Eigen::VectorXd& malleability_in)
              : macro_actions_(macro_actions_in),
                names_(names_in),
                hmm_(HMM(macro_actions_in.size())),
                goal_eps_(goal_eps_in),
                goal_bias_(goal_bias_in),
                trunc_min_(trunc_min_in),
                trunc_max_(trunc_max_in),
                count_penalty_(count_penalty_in),
                malleability_(malleability_in) {
}

MAD_RT::MAD_RT(const std::vector<MacroAction>& macro_actions_in,
               const std::vector<std::string>& names_in,
               const Eigen::MatrixXd& init_transition_counts,
               double goal_eps_in,
               double goal_bias_in,
               double trunc_min_in,
               double trunc_max_in,
               double count_penalty_in,
               const Eigen::VectorXd& malleability_in)
              : macro_actions_(macro_actions_in),
                names_(names_in),
                hmm_(HMM(macro_actions_in.size(), init_transition_counts)),
                goal_eps_(goal_eps_in),
                goal_bias_(goal_bias_in),
                trunc_min_(trunc_min_in),
                trunc_max_(trunc_max_in),
                count_penalty_(count_penalty_in),
                malleability_(malleability_in) {
}

// Plans a path from start to goal
std::vector<Eigen::VectorXd> MAD_RT::plan(const Eigen::VectorXd& start,
                                          const Eigen::VectorXd& goal,
                                          double max_time) {
    std::chrono::steady_clock::time_point tick = std::chrono::steady_clock::now();

    nodes_.push_back(MAD_RT_Node(std::vector<Eigen::VectorXd>{start},
                                 std::numeric_limits<size_t>::max(),
                                 std::numeric_limits<size_t>::max()));
    node_weights_.push_back(CalcNodeWeight(nodes_[0]));

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
        std::cout << "[MAD_RT] Iteration " << i++ << '\n';
        std::cout << "[MAD_RT] " << nodes_.size() << " nodes in tree" << '\n';

        size_t node_id = SampleNode();
        // DEBUG
        std::cout << "[MAD_RT] Sampled node " << node_id << '\n';

        // TODO(rzfeng): actually do this
        Eigen::VectorXd observation_heuristic(macro_actions_.size());
        observation_heuristic << 1.0, 1.0;

        size_t action_type;
        // Use only observation heuristic if no previous action
        if(nodes_[node_id].parent == std::numeric_limits<size_t>::max()) {
            std::discrete_distribution<size_t> distribution(observation_heuristic.begin(),
                                                            observation_heuristic.end());
            action_type = distribution(rng);
        }
        else {
            // action_type = hmm_.SampleMacroAction(nodes_[nodes_[node_id].parent].action.type,
            //                                      observation_heuristic);
            action_type = hmm_.SampleMacroAction(nodes_[node_id].action.type,
                                                 observation_heuristic);
        }
        // DEBUG
        std::cout << "[MAD_RT] Sampled action " << names_[action_type] << '\n';

        std::vector<Eigen::VectorXd> path;
        bool sample_goal = ((double)rand() / (double)RAND_MAX) < goal_bias_;
        if(sample_goal) {
            // DEBUG
            std::cout << "[MAD_RT] Targeting goal" << '\n';

            auto shear_shift = CalcShearShift(macro_actions_[action_type].path,
                                              nodes_[node_id].action.path.back(),
                                              goal);
            path = Morph(macro_actions_[action_type].path, shear_shift[0], shear_shift[1]);
        }
        else {
            // DEBUG
            std::cout << "[MAD_RT] Expanding tree" << '\n';

            // Truncate macro-action
            std::uniform_real_distribution<double> distribution(trunc_min_, trunc_max_);
            double trunc = distribution(rng);
            size_t trunc_size = ceil(macro_actions_[action_type].path.size() * trunc);
            if(trunc_size < 2) trunc_size = 2;

            path = macro_actions_[action_type].path;
            path.resize(trunc_size);

            auto shear_shift = SampleShearShift(path, nodes_[node_id].action.path.back(), trunc);
            path = Morph(path, shear_shift[0], shear_shift[1]);
        }

        // TODO(rzfeng): collision and joint limit check

        // TODO(rzfeng): put all this in an if for collision check
        nodes_.push_back(MAD_RT_Node(path, action_type, node_id));
        node_weights_.push_back(1.0);

        if(CalcDistance(path.back(), goal) < goal_eps_) {
            // DEBUG
            std::cout << "[MAD_RT] Found a path!" << '\n';

            std::vector<Eigen::VectorXd> motion_plan;
            size_t current_id = nodes_.size()-1;
            while(nodes_[current_id].parent != std::numeric_limits<size_t>::max()) {
                // Macro-actions are forward but overall path is backwards, so reverse macro-actions
                motion_plan.insert(motion_plan.end(), nodes_[current_id].action.path.rbegin(),
                                   nodes_[current_id].action.path.rend());
                current_id = nodes_[current_id].parent;
            }
            // We don't have to add the start node, since macro-actions are stored with the end node
            std::reverse(motion_plan.begin(), motion_plan.end());
            return motion_plan;
        }

        std::chrono::steady_clock::time_point tock = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = tock - tick;
        elapsed = diff.count();
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
        // TODO(rzfeng): clamp radians
        double t = double(i) / double(path.size()-1);
        morphed.push_back(path[i] + t*shear + shift);
    }
    return morphed;
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
    size_t node_id = distribution(rng);
    ++nodes_[node_id].sample_count;
    node_weights_[node_id] = CalcNodeWeight(nodes_[node_id]);
    return node_id;
}

// Computes the distance between two configurations
double MAD_RT::CalcDistance(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) const {
    return (q1 - q2).norm();
}