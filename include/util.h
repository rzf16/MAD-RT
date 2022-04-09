#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

// TODO(rzfeng): change
const size_t kNumDofs = 2;

struct MacroAction {
    std::vector<Eigen::VectorXd> path;
    size_t type;

    MacroAction(const std::vector<Eigen::VectorXd>& path_in, size_t type_in)
               : path(path_in), type(type_in) {}
};

inline double ClampRadians(double theta) {
    theta = fmod(theta, 2.0*M_PI);
    if(theta < -M_PI) theta += 2.0*M_PI;
    else if(theta >= M_PI) theta -= 2.0*M_PI;
    return theta;
}

#endif // UTIL_H