#include <iostream>
#include <vector>
#include <utility>
#include <Eigen/Dense>
#include "nn_struct.h"
#include "util.h"


NNStruct::NNStruct(size_t rebuild_threshold_in) : rebuild_threshold_(rebuild_threshold_in),
                                                  tree_initialized_(false) {}

void NNStruct::Add(const Eigen::VectorXd& pt, size_t id) {
    overflow_.push_back(std::make_pair(pt, id));

    // Rebuild tree if size of overflow buffer > threshold
    if(overflow_.size() > rebuild_threshold_) {
        auto pt_index = tree_.get_points();
        pointVec pts;
        for(const auto& pt : pt_index) {
            pts.push_back(pt.first);
        }
        // Convert overflow buffer to STL vectors
        for(const auto& q : overflow_) {
            std::vector<double> pt(q.first.data(), q.first.data() + q.first.size());
            pts.push_back(pt);
        }
        tree_ = KDTree(pts);
        tree_initialized_ = true;
        overflow_.clear();
    }
}

std::pair<Eigen::VectorXd, size_t> NNStruct::GetNearest(const Eigen::VectorXd& pt) {
    Eigen::VectorXd best_q(pt.size());
    double best_dist = 35383538.0;
    size_t best_idx = 35383538;

    // Get nearest in k-d tree
    if(tree_initialized_) {
        point_t pt_stl(pt.data(), pt.data() + pt.size());
        pointIndex best = tree_.nearest_pointIndex(pt_stl);
        best_q = Eigen::Map<Eigen::VectorXd>(best.first.data(), best.first.size());
        best_dist = L2(best_q, pt);
        best_idx = best.second;
    }

    // Check overflow buffer
    for(const auto& q : overflow_) {
        double dist = L2(q.first, pt);
        if(dist < best_dist) {
            best_q = q.first;
            best_dist = dist;
            best_idx = q.second;
        }
    }

    return std::make_pair(best_q, best_idx);
}