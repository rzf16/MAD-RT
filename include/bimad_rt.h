#ifndef BI_MAD_RT_H
#define BI_MAD_RT_H

#include <array>
#include <vector>
#include <string>
#include <random>
#include <Eigen/Dense>
#include "hmm.h"
#include "state_validity.h"
#include "util.h"
#include "observation.h"
#include "nn_struct.h"

class BiMAD_RT {
public:
    // TODO(rzfeng): decide if we want to truncate by proportion or length
    BiMAD_RT(const std::vector<MacroAction>& macro_actions_in,
             const std::vector<std::string>& names_in,
             StateValidityChecker* const state_validity_checker_in,
             ObservationHeuristic* observation_heuristic_in,
             double trunc_min_in=0.4,
             double trunc_max_in=1.0,
             double count_penalty_in=100.0,
             const Eigen::VectorXd& malleability_in=Eigen::VectorXd::Constant(kNumDofs, 0.5));

    BiMAD_RT(const std::vector<MacroAction>& macro_actions_in,
             const std::vector<std::string>& names_in,
             StateValidityChecker* const state_validity_checker_in,
             ObservationHeuristic* observation_heuristic_in,
             const Eigen::MatrixXd& init_transition_counts,
             double trunc_min_in=0.4,
             double trunc_max_in=1.0,
             double count_penalty_in=100.0,
             const Eigen::VectorXd& malleability_in=Eigen::VectorXd::Constant(kNumDofs, 0.5));

    // Plans a path from start to goal
    std::vector<Eigen::VectorXd> plan(const Eigen::VectorXd& start,
                                      const Eigen::VectorXd& goal,
                                      double max_time=300.0);

private:
    struct BiMAD_RT_Node {
        MacroAction action;
        size_t parent;
        size_t sample_count;

        BiMAD_RT_Node(const std::vector<Eigen::VectorXd>& path_in, size_t action_type_in,
                      size_t parent_in) : action(MacroAction(path_in, action_type_in)),
                                          parent(parent_in), sample_count(0) {}
    };

    // Morphs a path using shear and shift coefficients
    std::vector<Eigen::VectorXd> Morph(const std::vector<Eigen::VectorXd>& path,
                                       const Eigen::VectorXd& shear,
                                       const Eigen::VectorXd& shift) const;

    // Samples a shear coefficient and computes a shift coefficient
    std::array<Eigen::VectorXd, 2> SampleShearShift(const std::vector<Eigen::VectorXd>& path,
                                                    const Eigen::VectorXd& start,
                                                    double trunc) const;

    // Computes shear and shift coefficients to reach a target
    std::array<Eigen::VectorXd, 2> CalcShearShift(const std::vector<Eigen::VectorXd>& path,
                                                  const Eigen::VectorXd& start,
                                                  const Eigen::VectorXd& target) const;

    // Computes sampling weight for a node
    double CalcNodeWeight(const BiMAD_RT_Node& node) const;

    // Samples a node from the tree
    size_t SampleNode(bool start_tree);

    const std::vector<MacroAction> macro_actions_;
    std::vector<MacroAction> reversed_macro_actions_;
    const std::vector<std::string> names_;

    StateValidityChecker* const state_validity_checker_;
    ObservationHeuristic* observation_heuristic_;

    std::vector<BiMAD_RT_Node> start_nodes_;
    std::vector<double> start_node_weights_;
    NNStruct start_nn_;
    std::vector<BiMAD_RT_Node> goal_nodes_;
    std::vector<double> goal_node_weights_;
    NNStruct goal_nn_;

    HMM hmm_;

    double trunc_min_;
    double trunc_max_;
    double count_penalty_;
    Eigen::VectorXd malleability_;

    std::default_random_engine rng_;
};

#endif // BI_MAD_RT_H