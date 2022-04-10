#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

const size_t kNumDofs = 7;

void PrintPath(const std::vector<Eigen::VectorXd>&path);

double ClampRadians(double theta);

bool StrEndsWith(const std::string &str, const std::string& ending);

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

#endif // UTIL_H