#include <iostream>
#include <fstream>
#include <dirent.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "mad_rt.h"
#include "util.h"

int main() {
    std::vector<MacroAction> macro_actions;

    std::vector<Eigen::VectorXd> demo;
    Eigen::VectorXd pt(2);
    pt << 0.0, 0.0;
    demo.push_back(pt);
    pt(0) = 1.0;
    demo.push_back(pt);
    pt(0) = 2.0;
    demo.push_back(pt);
    pt(0) = 3.0;
    demo.push_back(pt);
    pt(0) = 4.0;
    demo.push_back(pt);
    macro_actions.push_back(MacroAction(demo, 0));

    pt(0) = 0.0;
    pt(1) = 0.0;
    demo[0] = pt;
    pt(1) = 1.0;
    demo[1] = pt;
    pt(1) = 2.0;
    demo[2] = pt;
    pt(1) = 3.0;
    demo[3] = pt;
    pt(1) = 4.0;
    demo[4] = pt;
    macro_actions.push_back(MacroAction(demo, 0));

    std::vector<std::string> names = {"X", "Y"};

    MAD_RT planner = MAD_RT(macro_actions, names);

    Eigen::VectorXd start(2);
    start << 2.0, 3.0;
    Eigen::VectorXd goal(2);
    goal << 35.0, 38.0;

    auto path = planner.plan(start, goal);
    for(const auto &q : path) {
        std::cout << q << '\n' << '\n';
    }
}