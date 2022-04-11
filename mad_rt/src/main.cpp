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
#include "smoothing.h"
#include "state_validity.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

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

int main(int argc, char** argv) {
    // Init ROS
    ros::init(argc, argv, "madrt");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Initialize Planning Scene Monitor
    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    psm->requestPlanningSceneState("/get_planning_scene");
    psm->startSceneMonitor("/move_group/monitored_planning_scene");

    // Init environment obstacles
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();
    collision_object.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.0;
    primitive.dimensions[primitive.BOX_Z] = 0.2;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.15;
    box_pose.position.y = 0.0;
    box_pose.position.z = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group_interface.getPlanningFrame();
    collision_object2.id = "box2";
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[primitive2.BOX_X] = 0.1;
    primitive2.dimensions[primitive2.BOX_Y] = 0.2;
    primitive2.dimensions[primitive2.BOX_Z] = 0.2;
    geometry_msgs::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x = -0.35;
    box_pose2.position.y = 0.0;
    box_pose2.position.z = 0.5;

    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_object2.operation = collision_object2.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    collision_objects.push_back(collision_object2);
    planning_scene_interface.addCollisionObjects(collision_objects);

    sleep(2.0);


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

    // Update planning scene

    psm->requestPlanningSceneState("/get_planning_scene");
    planning_scene::PlanningScenePtr planning_scene = psm->getPlanningScene();

    // Initialized planning scene monitor

    StateValidityChecker state_validity_checker(planning_scene, joint_model_group);

    // Initialize MAD_RT

    MAD_RT planner(macro_actions, names, &state_validity_checker);

    Eigen::VectorXd start(7);
    // start << 0.074, -1.13, -1.4, -1.08, -1.21, 1.29, 1.22;
    // start << 0,0,0,0,0,0,0;
    // start << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
    start << 0 , -0.785, 0, -2.356, 0, 1.571, 0.785;

    std::vector<double> joint_group_positions;
    joint_group_positions.push_back(0.0);
    joint_group_positions.push_back(-0.785);
    joint_group_positions.push_back(0.0);
    joint_group_positions.push_back(-2.356);
    joint_group_positions.push_back(0);
    joint_group_positions.push_back(1.571);
    joint_group_positions.push_back(0.785);

    if(!state_validity_checker.CheckValidity(start))
    {
        std::cout << "Invalid Start" << std::endl;
        return 0;
    }

    std::cout << state_validity_checker.CheckValidity(start) << std::endl;

    move_group_interface.setJointValueTarget(joint_group_positions);
    move_group_interface.move();


    Eigen::VectorXd goal(7);
    goal << -1.114, -2.056, -0.3538, -1.48, 1.18, 1.678, 2.54;
    // goal << 0.07452197849980702,-1.1334201864761648,-1.4754646390263484,-1.0818732297112024,-1.2166551688908482,1.291164571285692,1.227302802458642;

    if(!state_validity_checker.CheckValidity(goal))
    {
        std::cout << "Invalid Start" << std::endl;
        return 0;
    }

    auto path = planner.plan(start, goal);
    std::cout << "Unsmoothed length: " << CalcPathLength(path) << '\n';

    ShortcutSmoother smoother(&state_validity_checker);
    smoother.Smooth(path);
    std::cout << "Smoothed length: " << CalcPathLength(path) << '\n';

    moveit_msgs::RobotTrajectory madrt_traj;
    madrt_traj.joint_trajectory.joint_names = move_group_interface.getJointNames();

    std::cout << move_group_interface.getJointNames()[0] << std::endl;
  
    int count = 0;
    for(auto &config: path){
        count++;
        trajectory_msgs::JointTrajectoryPoint pt;
        for(int i=0; i < config.size(); i++){
            pt.positions.push_back(config(i));
            pt.time_from_start = ros::Duration(count);
        }
        madrt_traj.joint_trajectory.points.push_back(pt);
    }

    move_group_interface.execute(madrt_traj);

    PrintPath(path);
}