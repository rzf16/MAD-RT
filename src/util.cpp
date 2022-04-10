#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "util.h"

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

double ClampRadians(double theta) {
    theta = fmod(theta, 2.0*M_PI);
    if(theta < -M_PI) theta += 2.0*M_PI;
    else if(theta >= M_PI) theta -= 2.0*M_PI;
    return theta;
}

bool StrEndsWith(const std::string &str, const std::string& ending) {
    if (str.length() >= ending.length()) {
        return str.compare (str.length() - ending.length(), ending.length(), ending) == 0;
    } else {
        return false;
    }
}