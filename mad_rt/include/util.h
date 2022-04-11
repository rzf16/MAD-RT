#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

const size_t kNumDofs = 7;

// Computes the distance between two configurations
double CalcDistance(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2);

// Prints a path
void PrintPath(const std::vector<Eigen::VectorXd>&path);

// Computes the length of a path
double CalcPathLength(const std::vector<Eigen::VectorXd>& path);

// Clamps to [-pi, pi]
double ClampRadians(double theta);

// Checks if a string ends with ending
bool StrEndsWith(const std::string &str, const std::string& ending);

struct MacroAction {
    std::vector<Eigen::VectorXd> path;
    size_t type;

    MacroAction(const std::vector<Eigen::VectorXd>& path_in, size_t type_in)
               : path(path_in), type(type_in) {}

    // Prints the path of a macro-action
    void Print() {
        std::cout << "Macro-Action " << type << ":" << '\n';
        PrintPath(path);
    }
};

#endif // UTIL_H