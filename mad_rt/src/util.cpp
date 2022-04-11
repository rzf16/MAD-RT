#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "util.h"

// Computes the distance between two configurations
double CalcDistance(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) {
    return (q1 - q2).norm();
}

// Prints a path
void PrintPath(const std::vector<Eigen::VectorXd>&path) {
    for(const auto& q : path) {
        std::cout << "(";
        for(size_t i = 0; i < q.size(); ++i) {
            std::cout << q(i);
            if(i < q.size()-1) std::cout << ", ";
        }
        std::cout << ")\n";
    }
}

// Computes the length of a path
double CalcPathLength(const std::vector<Eigen::VectorXd>& path) {
    double length = 0.0;
    for(size_t i = 0; i < path.size()-1; ++i) {
        length += CalcDistance(path[i+1], path[i]);
    }
    return length;
}

// Clamps to [-pi, pi]
double ClampRadians(double theta) {
    theta = fmod(theta, 2.0*M_PI);
    if(theta < -M_PI) theta += 2.0*M_PI;
    else if(theta >= M_PI) theta -= 2.0*M_PI;
    return theta;
}

// Checks if a string ends with ending
bool StrEndsWith(const std::string &str, const std::string& ending) {
    if (str.length() >= ending.length()) {
        return str.compare (str.length() - ending.length(), ending.length(), ending) == 0;
    } else {
        return false;
    }
}