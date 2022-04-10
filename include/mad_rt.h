#ifndef MAD_RT_H
#define MAD_RT_H

#include <array>
#include <vector>
#include <string>
#include <random>
#include <Eigen/Dense>
#include "hmm.h"
#include "state_validity.h"
#include "util.h"

class MAD_RT {
public:
    // TODO(rzfeng): decide if we want to truncate by proportion or length
    MAD_RT(const std::vector<MacroAction>& macro_actions_in,
           const std::vector<std::string>& names_in,
           // TODO(tweiheng): pass in pointers to planning scene & joint model group
           double goal_eps_in=0.01,
           double goal_bias_in=0.05,
           double trunc_min_in=0.4,
           double trunc_max_in=1.0,
           double count_penalty_in=100.0,
           const Eigen::VectorXd& malleability_in=Eigen::VectorXd::Constant(kNumDofs, 0.5));

    MAD_RT(const std::vector<MacroAction>& macro_actions_in,
           const std::vector<std::string>& names_in,
           // TODO(tweiheng): pass in pointers to planning scene & joint model group
           const Eigen::MatrixXd& init_transition_counts,
           double goal_eps_in=0.01,
           double goal_bias_in=0.05,
           double trunc_min_in=0.4,
           double trunc_max_in=1.0,
           double count_penalty_in=100.0,
           const Eigen::VectorXd& malleability_in=Eigen::VectorXd::Constant(kNumDofs, 0.5));

    // Plans a path from start to goal
    std::vector<Eigen::VectorXd> plan(const Eigen::VectorXd& start,
                                      const Eigen::VectorXd& goal,
                                      double max_time=300.0);

private:
    struct MAD_RT_Node {
        MacroAction action;
        size_t parent;
        size_t sample_count;

        MAD_RT_Node(const std::vector<Eigen::VectorXd>& path_in, size_t action_type_in,
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
    double CalcNodeWeight(const MAD_RT_Node& node) const;

    // Samples a node from the tree
    size_t SampleNode();

    // Computes the distance between two configurations
    double CalcDistance(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) const;

    const std::vector<MacroAction> macro_actions_;
    const std::vector<std::string> names_;

    StateValidityChecker state_validity_checker_;

    // TODO(rzfeng): could be super inefficient, may need to replace with a better data structure
    std::vector<MAD_RT_Node> nodes_;
    std::vector<double> node_weights_;

    HMM hmm_;

    double goal_eps_;
    double goal_bias_;
    double trunc_min_;
    double trunc_max_;
    double count_penalty_;
    Eigen::VectorXd malleability_;

    std::default_random_engine rng;
};

#endif // MAD_RT_H