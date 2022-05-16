#ifndef NN_STRUCT_H
#define NN_STRUCT_H

#include <vector>
#include <utility>
#include <Eigen/Dense>
#include "kd_tree.h"


class NNStruct {
public:
    NNStruct(size_t rebuild_threshold_in=300);

    void Add(const Eigen::VectorXd& pt, size_t id);

    std::pair<Eigen::VectorXd, size_t> GetNearest(const Eigen::VectorXd& pt);

private:
    KDTree tree_;
    std::vector<std::pair<Eigen::VectorXd, size_t>> overflow_;

    size_t rebuild_threshold_;
    bool tree_initialized_;
};


#endif // NN_STRUCT_H