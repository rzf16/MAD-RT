#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

const size_t kNumDofs = 7;

inline void PrintPath(const std::vector<Eigen::VectorXd>&path) {
    for(const auto& q : path) {
        std::cout << "(";
        for(size_t i = 0; i < q.size(); ++i) {
            std::cout << q(i);
            if(i < q.size()-1) std::cout << ", ";
        }
        std::cout << ")\n";
    }
}

struct MacroAction {
    std::vector<Eigen::VectorXd> path;
    size_t type;

    MacroAction(const std::vector<Eigen::VectorXd>& path_in, size_t type_in)
               : path(path_in), type(type_in) {}

    void Print() {
        std::cout << "Macro-Action " << type << ":" << '\n';
        PrintPath(path);
    }
};

inline double ClampRadians(double theta) {
    theta = fmod(theta, 2.0*M_PI);
    if(theta < -M_PI) theta += 2.0*M_PI;
    else if(theta >= M_PI) theta -= 2.0*M_PI;
    return theta;
}

inline bool StrEndsWith(const std::string &str, const std::string& ending) {
    if (str.length() >= ending.length()) {
        return str.compare (str.length() - ending.length(), ending.length(), ending) == 0;
    } else {
        return false;
    }
}

#endif // UTIL_H