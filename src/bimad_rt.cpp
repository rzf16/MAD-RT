#include <iostream>
#include <math.h>
#include <algorithm>
#include <chrono>
#include <random>
#include <limits>
#include "bimad_rt.h"
#include "state_validity.h"
#include "observation.h"

BiMAD_RT::BiMAD_RT(const std::vector<MacroAction>& macro_actions_in,
                   const std::vector<std::string>& names_in,
                   StateValidityChecker* const state_validity_checker_in,
                   ObservationHeuristic* observation_heuristic_in,
                   double trunc_min_in,
                   double trunc_max_in,
                   double count_penalty_in,
                   const Eigen::VectorXd& malleability_in)
                  : macro_actions_(macro_actions_in),
                    names_(names_in),
                    state_validity_checker_(state_validity_checker_in),
                    observation_heuristic_(observation_heuristic_in),
                    hmm_(HMM(macro_actions_in.size())),
                    trunc_min_(trunc_min_in),
                    trunc_max_(trunc_max_in),
                    count_penalty_(count_penalty_in),
                    malleability_(malleability_in) {
    for(const auto& action : macro_actions_) {
        MacroAction reversed = action;
        std::reverse(reversed.path.begin(), reversed.path.end());
        reversed_macro_actions_.push_back(reversed);
    }
}

BiMAD_RT::BiMAD_RT(const std::vector<MacroAction>& macro_actions_in,
                   const std::vector<std::string>& names_in,
                   StateValidityChecker* const state_validity_checker_in,
                   ObservationHeuristic* observation_heuristic_in,
                   const Eigen::MatrixXd& init_transition_counts,
                   double trunc_min_in,
                   double trunc_max_in,
                   double count_penalty_in,
                   const Eigen::VectorXd& malleability_in)
                  : macro_actions_(macro_actions_in),
                    names_(names_in),
                    state_validity_checker_(state_validity_checker_in),
                    observation_heuristic_(observation_heuristic_in),
                    hmm_(HMM(macro_actions_in.size(), init_transition_counts)),
                    trunc_min_(trunc_min_in),
                    trunc_max_(trunc_max_in),
                    count_penalty_(count_penalty_in),
                    malleability_(malleability_in) {
    for(const auto& action : macro_actions_) {
        MacroAction reversed = action;
        std::reverse(reversed.path.begin(), reversed.path.end());
        reversed_macro_actions_.push_back(reversed);
    }
}

// Plans a path from start to goal
std::vector<Eigen::VectorXd> BiMAD_RT::plan(const Eigen::VectorXd& start,
                                            const Eigen::VectorXd& goal,
                                            double max_time) {
    observation_heuristic_->SetProblem(start, goal);
    std::chrono::steady_clock::time_point tick = std::chrono::steady_clock::now();

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

    start_nodes_.push_back(BiMAD_RT_Node(std::vector<Eigen::VectorXd>{start},
                                         std::numeric_limits<size_t>::max(),
                                         std::numeric_limits<size_t>::max()));
    start_node_weights_.push_back(CalcNodeWeight(start_nodes_[0]));
    start_nn_.Add(start, 0);

    goal_nodes_.push_back(BiMAD_RT_Node(std::vector<Eigen::VectorXd>{goal},
                                         std::numeric_limits<size_t>::max(),
                                         std::numeric_limits<size_t>::max()));
    goal_node_weights_.push_back(CalcNodeWeight(goal_nodes_[0]));
    goal_nn_.Add(goal, 0);

    // While time not exhausted:
    //     Sample a node from expansion tree
    //     Get observation heuristic for current node
    //     Get a macro-action from HMM using current action and observation
    //     Truncate macro-action
    //     Morph macro-action
    //     Check for collisions and joint limits
    //     Add edge to expansion tree
    //     Get nearest node in connection tree
    //     Try to connect the two nodes
    //     If goal reached: backtrack
    // Return failure

    double elapsed = 0.0;
    size_t i = 0;
    bool expand_start = true;
    while(elapsed < max_time) {
        // DEBUG
        // std::cout << "[BiMAD_RT] Iteration " << i++ << '\n';
        // std::cout << "[BiMAD_RT] " << start_nodes_.size() << " nodes in start tree" << '\n';
        // std::cout << "[BiMAD_RT] " << goal_nodes_.size() << " nodes in goal tree" << '\n';

        size_t node_id = SampleNode(expand_start);
        // DEBUG
        // if(expand_start) std::cout << "[BiMAD_RT] Sampled node " << node_id <<
                                      // " of start tree" << '\n';
        // else std::cout << "[BiMAD_RT] Sampled node " << node_id <<
                          // " of goal tree" << '\n';

        Eigen::VectorXd observation_weights(macro_actions_.size());

        for(size_t j = 0; j < macro_actions_.size(); ++j) {
            if(expand_start) {
                observation_heuristic_->SetProblem(start, goal);
                observation_weights(j) = observation_heuristic_->Compute(
                                            start_nodes_[node_id].action.path.back(),
                                            macro_actions_[j]
                                         );
            }
            else {
                observation_heuristic_->SetProblem(goal, start);
                observation_weights(j) = observation_heuristic_->Compute(
                                            goal_nodes_[node_id].action.path.back(),
                                            reversed_macro_actions_[j]
                                         );
            }
        }
        // Normalize the weights
        observation_weights = observation_weights / observation_weights.maxCoeff();

        // Not using transition probabilitise
        std::discrete_distribution<size_t> obs_distribution(observation_weights.data(),
                                                            observation_weights.data() + observation_weights.size());
        size_t action_type = obs_distribution(rng_);
        // DEBUG
        // std::cout << "[BiMAD_RT] Sampled action " << names_[action_type] << '\n';

        // DEBUG
        // if(expand_start) std::cout << "[BiMAD_RT] Expanding start tree" << '\n';
        // else std::cout << "[BiMAD_RT] Expanding goal tree" << '\n';

        // Truncate macro-action
        std::uniform_real_distribution<double> trunc_distribution(trunc_min_, trunc_max_);
        double trunc = trunc_distribution(rng_);
        size_t trunc_size = ceil(macro_actions_[action_type].path.size() * trunc);
        if(trunc_size < 2) trunc_size = 2;

        std::vector<Eigen::VectorXd> path = expand_start ?
                                            macro_actions_[action_type].path :
                                            reversed_macro_actions_[action_type].path;
        path.resize(trunc_size);

        auto shear_shift = expand_start ?
                           SampleShearShift(path, start_nodes_[node_id].action.path.back(), trunc) :
                           SampleShearShift(path, goal_nodes_[node_id].action.path.back(), trunc);
        path = Morph(path, shear_shift[0], shear_shift[1]);

        bool valid = true;
        // TODO(rzfeng): check with interpolation
        for(const auto& q : path) {
            if(!state_validity_checker_->CheckValidity(q)) {
                valid = false;
                // std::cout << "[BiMAD_RT] Invalid state!" << '\n';
                break;
            }
        }

        if(valid) {
            if(expand_start) {
                start_nodes_.push_back(BiMAD_RT_Node(path, action_type, node_id));
                start_node_weights_.push_back(1.0);
                start_nn_.Add(path.back(), start_nodes_.size()-1);

                // Connection
                // DEBUG
                // std::cout << "[BiMAD_RT] Attempting connection from goal tree" << '\n';
                auto nn = goal_nn_.GetNearest(path.back());

                // Select best macro-action for connection
                observation_heuristic_->SetProblem(nn.first, path.back());
                for(size_t j = 0; j < macro_actions_.size(); ++j) {
                    observation_weights(j) = observation_heuristic_->Compute(
                                                nn.first,
                                                reversed_macro_actions_[j]
                                             );
                }
                observation_weights.maxCoeff(&action_type);

                shear_shift = CalcShearShift(reversed_macro_actions_[action_type].path,
                                             nn.first,
                                             path.back());
                auto connection = Morph(reversed_macro_actions_[action_type].path,
                                        shear_shift[0],
                                        shear_shift[1]);

                // TODO(rzfeng): check with interpolation
                bool connected = true;
                for(const auto& q : connection) {
                    if(!state_validity_checker_->CheckValidity(q)) {
                        connected = false;
                        break;
                    }
                }

                if(connected) {
                    std::chrono::steady_clock::time_point tock = std::chrono::steady_clock::now();
                    std::chrono::duration<double> diff = tock - tick;
                    elapsed = diff.count();
                    // DEBUG
                    std::cout << '\n';
                    std::cout << "[BiMAD_RT] Found a path in " << elapsed << " seconds!" << '\n';

                    std::vector<Eigen::VectorXd> motion_plan;
                    std::vector<size_t> macro_action_seq;

                    // Backtrack on start tree
                    size_t current_id = start_nodes_.size()-1;
                    while(start_nodes_[current_id].parent != std::numeric_limits<size_t>::max()) {
                        // Macro-actions are forward but overall path is backwards, so reverse macro-actions
                        motion_plan.insert(motion_plan.end(),
                                           start_nodes_[current_id].action.path.rbegin(),
                                           start_nodes_[current_id].action.path.rend());
                        macro_action_seq.push_back(start_nodes_[current_id].action.type);
                        current_id = start_nodes_[current_id].parent;
                    }
                    // We don't have to add the start node, since macro-actions are stored with the end node
                    std::reverse(motion_plan.begin(), motion_plan.end());
                    std::reverse(macro_action_seq.begin(), macro_action_seq.end());

                    // Add connection
                    motion_plan.insert(motion_plan.end(), connection.rbegin(),
                                       connection.rend());
                    macro_action_seq.push_back(action_type);

                    // Backtrack on goal tree
                    current_id = nn.second;
                    while(goal_nodes_[current_id].parent != std::numeric_limits<size_t>::max()) {
                        // Macro-actions are backward but overall path is forward, so reverse macro-actions
                        motion_plan.insert(motion_plan.end(),
                                           goal_nodes_[current_id].action.path.rbegin(),
                                           goal_nodes_[current_id].action.path.rend());
                        macro_action_seq.push_back(goal_nodes_[current_id].action.type);
                        current_id = goal_nodes_[current_id].parent;
                    }

                    // DEBUG
                    std::cout << "[BiMAD_RT] Macro-Action Sequence: ";
                    for(size_t i = 0; i < macro_action_seq.size(); ++i) {
                        std::cout << names_[macro_action_seq[i]];
                        if(i < macro_action_seq.size()-1) std::cout << ", ";
                    }
                    std::cout << '\n';

                    return motion_plan;
                }
            }
            else {
                goal_nodes_.push_back(BiMAD_RT_Node(path, action_type, node_id));
                goal_node_weights_.push_back(1.0);
                goal_nn_.Add(path.back(), goal_nodes_.size()-1);

                // Connection
                // DEBUG
                // std::cout << "[BiMAD_RT] Attempting connection from start tree" << '\n';
                auto nn = start_nn_.GetNearest(path.back());

                // Select best macro-action for connection
                observation_heuristic_->SetProblem(nn.first, path.back());
                for(size_t j = 0; j < macro_actions_.size(); ++j) {
                    observation_weights(j) = observation_heuristic_->Compute(
                                                nn.first,
                                                macro_actions_[j]
                                             );
                }
                observation_weights.maxCoeff(&action_type);

                shear_shift = CalcShearShift(macro_actions_[action_type].path,
                                             nn.first,
                                             path.back());
                auto connection = Morph(macro_actions_[action_type].path,
                                        shear_shift[0],
                                        shear_shift[1]);

                // TODO(rzfeng): check with interpolation
                bool connected = true;
                for(const auto& q : connection) {
                    if(!state_validity_checker_->CheckValidity(q)) {
                        connected = false;
                        break;
                    }
                }

                if(connected) {
                    std::chrono::steady_clock::time_point tock = std::chrono::steady_clock::now();
                    std::chrono::duration<double> diff = tock - tick;
                    elapsed = diff.count();
                    // DEBUG
                    std::cout << '\n';
                    std::cout << "[BiMAD_RT] Found a path in " << elapsed << " seconds!" << '\n';

                    std::vector<Eigen::VectorXd> motion_plan;
                    std::vector<size_t> macro_action_seq;

                    // Backtrack on start tree
                    size_t current_id = nn.second;
                    while(start_nodes_[current_id].parent != std::numeric_limits<size_t>::max()) {
                        // Macro-actions are forward but overall path is backwards, so reverse macro-actions
                        motion_plan.insert(motion_plan.end(),
                                           start_nodes_[current_id].action.path.rbegin(),
                                           start_nodes_[current_id].action.path.rend());
                        macro_action_seq.push_back(start_nodes_[current_id].action.type);
                        current_id = start_nodes_[current_id].parent;
                    }
                    // We don't have to add the start node, since macro-actions are stored with the end node
                    std::reverse(motion_plan.begin(), motion_plan.end());
                    std::reverse(macro_action_seq.begin(), macro_action_seq.end());

                    // Add connection
                    motion_plan.insert(motion_plan.end(), connection.begin(),
                                       connection.end());
                    macro_action_seq.push_back(action_type);

                    // Backtrack on goal tree
                    current_id = goal_nodes_.size()-1;
                    while(goal_nodes_[current_id].parent != std::numeric_limits<size_t>::max()) {
                        motion_plan.insert(motion_plan.end(),
                                           goal_nodes_[current_id].action.path.rbegin(),
                                           goal_nodes_[current_id].action.path.rend());
                        macro_action_seq.push_back(goal_nodes_[current_id].action.type);
                        current_id = goal_nodes_[current_id].parent;
                    }

                    // DEBUG
                    std::cout << "[BiMAD_RT] Macro-Action Sequence: ";
                    for(size_t i = 0; i < macro_action_seq.size(); ++i) {
                        std::cout << names_[macro_action_seq[i]];
                        if(i < macro_action_seq.size()-1) std::cout << ", ";
                    }
                    std::cout << '\n';

                    return motion_plan;
                }
            }
        }

        expand_start = !expand_start;

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
std::vector<Eigen::VectorXd> BiMAD_RT::Morph(const std::vector<Eigen::VectorXd>& path,
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
        double step_size = 0.1;
        Eigen::VectorXd step = (diff / diff.norm()) * step_size;
        size_t num_steps = floor(diff.norm() / step_size);
        for(size_t j = 0; j < num_steps; ++j) {
            interpolated.push_back(morphed[i] + step * j);
        }
    }
    interpolated.push_back(morphed.back());
    return interpolated;
}

// Samples a shear coefficient and computes a shift coefficient
std::array<Eigen::VectorXd, 2> BiMAD_RT::SampleShearShift(const std::vector<Eigen::VectorXd>& path,
                                                          const Eigen::VectorXd& start,
                                                          double trunc) const {
    Eigen::VectorXd shear = Eigen::VectorXd::Random(start.size());
    shear = shear.cwiseProduct(malleability_) * trunc;

    Eigen::VectorXd shift = start - path[0];

    return std::array<Eigen::VectorXd, 2> {{shear, shift}};
}

// Computes shear and shift coefficients to reach a target
std::array<Eigen::VectorXd, 2> BiMAD_RT::CalcShearShift(const std::vector<Eigen::VectorXd>& path,
                                                        const Eigen::VectorXd& start,
                                                        const Eigen::VectorXd& target) const {
    Eigen::VectorXd shift = start - path[0];
    Eigen::VectorXd shear = target - (path.back() + shift);

    return std::array<Eigen::VectorXd, 2> {{shear, shift}};
}

// Computes sampling weight for a node
double BiMAD_RT::CalcNodeWeight(const BiMAD_RT_Node& node) const {
    return 1.0 / (count_penalty_ * node.sample_count + 1.0);
}

// Samples a node from the tree
size_t BiMAD_RT::SampleNode(bool start_tree) {
    if(start_tree) {
        std::discrete_distribution<size_t> distribution(start_node_weights_.begin(),
                                                        start_node_weights_.end());
        size_t node_id = distribution(rng_);
        ++start_nodes_[node_id].sample_count;
        start_node_weights_[node_id] = CalcNodeWeight(start_nodes_[node_id]);
        return node_id;
    }
    else {
        std::discrete_distribution<size_t> distribution(goal_node_weights_.begin(),
                                                        goal_node_weights_.end());
        size_t node_id = distribution(rng_);
        ++goal_nodes_[node_id].sample_count;
        goal_node_weights_[node_id] = CalcNodeWeight(goal_nodes_[node_id]);
        return node_id;
    }
}
