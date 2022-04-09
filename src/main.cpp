#include <iostream>
#include <fstream>
#include <dirent.h>
#include <vector>
#include <string>
#include <sstream>
#include <utility>
#include <Eigen/Dense>
#include <ctime>
#include "mad_rt.h"
#include "util.h"

const std::string MACRO_ACTION_DIR = "config/macro_actions/";

std::pair<MacroAction, std::string> ReadMacroAction(const std::string& filename, size_t i) {
    std::string name = filename.substr(0, filename.length()-4);

    std::vector<Eigen::VectorXd> path;
    std::fstream file(MACRO_ACTION_DIR+filename, std::ios::in);
    if(file.is_open()) {
        std::string line;
        while(getline(file, line)) {
            Eigen::VectorXd q(kNumDofs);
            std::string word;
            std::stringstream str(line);
            size_t i = 0;
            while(getline(str, word, ',')) {
                q(i++) = std::stod(word);
            }
            path.push_back(q);
        }
    }

    return std::make_pair(MacroAction(path, i), name);
}

int main() {
    // Seed randomness
    srand((unsigned int)time(NULL));

    std::vector<MacroAction> macro_actions;
    std::vector<std::string> names;

    // Read in macro-actions
    DIR *dir;
    struct dirent *ent;
    if((dir = opendir(MACRO_ACTION_DIR.c_str())) != NULL) {
        size_t i = 0;
        while((ent = readdir (dir)) != NULL) {
            std::string filename(ent->d_name);
            if(StrEndsWith(filename, ".csv")) {
                auto info = ReadMacroAction(filename, i++);
                macro_actions.push_back(info.first);
                names.push_back(info.second);
            }
        }
        closedir(dir);
    }
    else {
        std::cout << "[Main] Failed to open macro-action directory" << '\n';
        return 1;
    }

    MAD_RT planner(macro_actions, names);

    Eigen::VectorXd start(7);
    start << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Eigen::VectorXd goal(7);
    goal << 35.0, 38.0, 3.3, 14.8, 1.18, 16.78, 25.4;

    auto path = planner.plan(start, goal);
    PrintPath(path);
}