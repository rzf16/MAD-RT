#include <iostream>
#include <fstream>
#include <dirent.h>
#include <vector>
#include <string>
#include <sstream>
#include <utility>
#include <Eigen/Dense>
#include <ctime>
#include "bimad_rt.h"
#include "mad_rt.h"
#include "util.h"
#include "smoothing.h"
#include "state_validity.h"

#include <chrono>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

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

    // Init Start
    Eigen::VectorXd start(7);
    // start << 0.074, -1.13, -1.4, -1.08, -1.21, 1.29, 1.22;
    // start << 0,0,0,0,0,0,0;
    // start << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
    // start << 0 , 0, 0, -1.343, 0, 1.413, 0.785;
    start << -2.7 , 0, 0, -1.4, 0, 1.4, 0.785; // Top shelf
    // start << 0, -1.396, -0.140, -2.618, -0.140, 1.239, -2.112; // pre-hole
    // start << -1.45, -0.175, -0.175, -1.920, -0.017, 1.745, 2.461;
    // start << 1.57, -0.91, 0, -2.67, -0.03, 1.76, 0; // slot goal


    std::vector<double> start_vec(start.data(), start.data() + start.size());


    // Move arm to start position
    move_group_interface.setJointValueTarget(start_vec);
    move_group_interface.move();

    // Init environment obstacles
    moveit_msgs::CollisionObject shelf1, shelf2, shelf3, shelf4, shelf5, shelf6;
    moveit_msgs::CollisionObject shelf_side1, shelf_side2, shelf_side3, shelf_side4;
    moveit_msgs::CollisionObject shelf_back1, shelf_back2;
    moveit_msgs::CollisionObject wall1, wall2, wall3, wall4, wall5, wall6, wall7;

    shelf1.header.frame_id = move_group_interface.getPlanningFrame();
    shelf2.header.frame_id = move_group_interface.getPlanningFrame();
    shelf3.header.frame_id = move_group_interface.getPlanningFrame();
    shelf4.header.frame_id = move_group_interface.getPlanningFrame();
    shelf5.header.frame_id = move_group_interface.getPlanningFrame();
    shelf6.header.frame_id = move_group_interface.getPlanningFrame();

    shelf_side1.header.frame_id = move_group_interface.getPlanningFrame();
    shelf_side2.header.frame_id = move_group_interface.getPlanningFrame();
    shelf_side3.header.frame_id = move_group_interface.getPlanningFrame();
    shelf_side4.header.frame_id = move_group_interface.getPlanningFrame();

    shelf_back1.header.frame_id = move_group_interface.getPlanningFrame();
    shelf_back2.header.frame_id = move_group_interface.getPlanningFrame();

    wall1.header.frame_id = move_group_interface.getPlanningFrame();
    wall2.header.frame_id = move_group_interface.getPlanningFrame();
    wall3.header.frame_id = move_group_interface.getPlanningFrame();
    wall4.header.frame_id = move_group_interface.getPlanningFrame();

    wall5.header.frame_id = move_group_interface.getPlanningFrame();
    wall6.header.frame_id = move_group_interface.getPlanningFrame();
    wall7.header.frame_id = move_group_interface.getPlanningFrame();

    shelf_side1.id = "ss1";
    shelf_side2.id = "ss2";
    shelf_side3.id = "ss3";
    shelf_side4.id = "ss4";

    shelf1.id = "shelf1";
    shelf2.id = "shelf2";
    shelf3.id = "shelf3";
    shelf4.id = "shelf4";
    shelf5.id = "shelf5";
    shelf6.id = "shelf6";

    shelf_back1.id = "sb1";
    shelf_back2.id = "sb2";

    wall1.id = "w1";
    wall2.id = "w2";
    wall3.id = "w3";
    wall4.id = "w4";

    wall5.id = "w5";
    wall6.id = "w6";
    wall7.id = "w7";

    shape_msgs::SolidPrimitive shelf, shelf_side, shelf_back, wall1p, wall2p, wall3p;
    shelf.type = shelf.BOX;
    shelf_side.type = shelf_side.BOX;
    shelf_back.type = shelf_back.BOX;
    wall1p.type = wall1p.BOX;
    wall2p.type = wall2p.BOX;
    wall3p.type = wall3p.BOX;

    shelf.dimensions.resize(3);
    shelf.dimensions[shelf.BOX_X] = 0.4;
    shelf.dimensions[shelf.BOX_Y] = 1.0;
    shelf.dimensions[shelf.BOX_Z] = 0.025;

    shelf_side.dimensions.resize(3);
    shelf_side.dimensions[shelf_side.BOX_X] = 0.4;
    shelf_side.dimensions[shelf_side.BOX_Y] = 0.025;
    shelf_side.dimensions[shelf_side.BOX_Z] = 1.0;    

    shelf_back.dimensions.resize(3);
    shelf_back.dimensions[shelf_back.BOX_X] = 0.025;
    shelf_back.dimensions[shelf_back.BOX_Y] = 1.0;
    shelf_back.dimensions[shelf_back.BOX_Z] = 1.0;

    wall1p.dimensions.resize(3);
    wall1p.dimensions[wall1p.BOX_X] = 0.1;
    wall1p.dimensions[wall1p.BOX_Y] = 2.0;
    wall1p.dimensions[wall1p.BOX_Z] = 0.4;

    wall2p.dimensions.resize(3);
    wall2p.dimensions[wall2p.BOX_X] = 0.1;
    wall2p.dimensions[wall2p.BOX_Y] = 0.75;
    wall2p.dimensions[wall2p.BOX_Z] = 0.5;

    wall3p.dimensions.resize(3);
    wall3p.dimensions[wall3p.BOX_X] = 0.1;
    wall3p.dimensions[wall3p.BOX_Y] = 0.75;
    wall3p.dimensions[wall3p.BOX_Z] = 0.3;
    
    geometry_msgs::Pose shelf_pose1, shelf_pose2, shelf_pose3, shelf_pose4, shelf_pose5, shelf_pose6;
    shelf_pose1.orientation.w = 1.0;
    shelf_pose1.position.x = -0.5;
    shelf_pose1.position.y = 0.0;
    shelf_pose1.position.z = 0.0;
    shelf_pose2.orientation.w = 1.0;
    shelf_pose2.position.x = -0.5;
    shelf_pose2.position.y = 0.0;
    shelf_pose2.position.z = 0.5;
    shelf_pose3.orientation.w = 1.0;
    shelf_pose3.position.x = -0.5;
    shelf_pose3.position.y = 0.0;
    shelf_pose3.position.z = 1.0;
    shelf_pose4.orientation.w = 1.0;
    shelf_pose4.position.x = 0.5;
    shelf_pose4.position.y = 0.0;
    shelf_pose4.position.z = 0.0;
    shelf_pose5.orientation.w = 1.0;
    shelf_pose5.position.x = 0.5;
    shelf_pose5.position.y = 0.0;
    shelf_pose5.position.z = 0.5;
    shelf_pose6.orientation.w = 1.0;
    shelf_pose6.position.x = 0.5;
    shelf_pose6.position.y = 0.0;
    shelf_pose6.position.z = 1.0;

    geometry_msgs::Pose shelf_side_pose1, shelf_side_pose2, shelf_side_pose3, shelf_side_pose4;
    shelf_side_pose1.orientation.w = 1.0;
    shelf_side_pose1.position.x = -0.5;
    shelf_side_pose1.position.y = -0.5;
    shelf_side_pose1.position.z = 0.5;    
    shelf_side_pose2.orientation.w = 1.0;
    shelf_side_pose2.position.x = 0.5;
    shelf_side_pose2.position.y = -0.5;
    shelf_side_pose2.position.z = 0.5;
    shelf_side_pose3.orientation.w = 1.0;
    shelf_side_pose3.position.x = -0.5;
    shelf_side_pose3.position.y = 0.5;
    shelf_side_pose3.position.z = 0.5;
    shelf_side_pose4.orientation.w = 1.0;
    shelf_side_pose4.position.x = 0.5;
    shelf_side_pose4.position.y = 0.5;
    shelf_side_pose4.position.z = 0.5;

    geometry_msgs::Pose shelf_back_pose1, shelf_back_pose2;
    shelf_back_pose1.orientation.w = 1.0;
    shelf_back_pose1.position.x = -0.7;
    shelf_back_pose1.position.y = 0.0;
    shelf_back_pose1.position.z = 0.5;
    shelf_back_pose2.orientation.w = 1.0;
    shelf_back_pose2.position.x = 0.7;
    shelf_back_pose2.position.y = 0.0;
    shelf_back_pose2.position.z = 0.5;

    geometry_msgs::Pose wall1_pose, wall2_pose, wall3_pose, wall4_pose;
    wall1_pose.orientation.w = 1.0;
    wall1_pose.position.x = 0.3;
    wall1_pose.position.y = 0.0;
    wall1_pose.position.z = 0.2;
    wall2_pose.orientation.w = 1.0;
    wall2_pose.position.x = 0.3;
    wall2_pose.position.y = 0.0;
    wall2_pose.position.z = 1.1;
    wall3_pose.orientation.w = 1.0;
    wall3_pose.position.x = 0.3;
    wall3_pose.position.y = -0.625;
    wall3_pose.position.z = 0.65;
    wall4_pose.orientation.w = 1.0;
    wall4_pose.position.x = 0.3;
    wall4_pose.position.y = 0.55;
    wall4_pose.position.z = 0.65;

    geometry_msgs::Pose wall5_pose, wall6_pose, wall7_pose;
    wall5_pose.orientation.w = 1.0;
    wall5_pose.position.x = 0.3;
    wall5_pose.position.y = -0.2;
    wall5_pose.position.z = 0.225;
    wall6_pose.orientation.w = 1.0;
    wall6_pose.position.x = 0.3;
    wall6_pose.position.y = -0.2;
    wall6_pose.position.z = 0.9;
    wall7_pose.orientation.w = 1.0;
    wall7_pose.position.x = 0.3;
    wall7_pose.position.y = -0.625;
    wall7_pose.position.z = 0.55;

    shelf1.primitives.push_back(shelf);
    shelf1.primitive_poses.push_back(shelf_pose1);
    shelf1.operation = shelf1.ADD;
    shelf2.primitives.push_back(shelf);
    shelf2.primitive_poses.push_back(shelf_pose2);
    shelf2.operation = shelf2.ADD;
    shelf3.primitives.push_back(shelf);
    shelf3.primitive_poses.push_back(shelf_pose3);
    shelf3.operation = shelf3.ADD;
    shelf4.primitives.push_back(shelf);
    shelf4.primitive_poses.push_back(shelf_pose4);
    shelf4.operation = shelf4.ADD;
    shelf5.primitives.push_back(shelf);
    shelf5.primitive_poses.push_back(shelf_pose5);
    shelf5.operation = shelf5.ADD;
    shelf6.primitives.push_back(shelf);
    shelf6.primitive_poses.push_back(shelf_pose6);
    shelf6.operation = shelf6.ADD;

    shelf_side1.primitives.push_back(shelf_side);
    shelf_side1.primitive_poses.push_back(shelf_side_pose1);
    shelf_side1.operation = shelf_side1.ADD;
    shelf_side2.primitives.push_back(shelf_side);
    shelf_side2.primitive_poses.push_back(shelf_side_pose2);
    shelf_side2.operation = shelf_side2.ADD;
    shelf_side3.primitives.push_back(shelf_side);
    shelf_side3.primitive_poses.push_back(shelf_side_pose3);
    shelf_side3.operation = shelf_side3.ADD;
    shelf_side4.primitives.push_back(shelf_side);
    shelf_side4.primitive_poses.push_back(shelf_side_pose4);
    shelf_side4.operation = shelf_side4.ADD;

    shelf_back1.primitives.push_back(shelf_back);
    shelf_back1.primitive_poses.push_back(shelf_back_pose1);
    shelf_back1.operation = shelf_back1.ADD;
    shelf_back2.primitives.push_back(shelf_back);
    shelf_back2.primitive_poses.push_back(shelf_back_pose2);
    shelf_back2.operation = shelf_back2.ADD;

    wall1.primitives.push_back(wall1p);
    wall1.primitive_poses.push_back(wall1_pose);
    wall1.operation = wall1.ADD;
    wall2.primitives.push_back(wall1p);
    wall2.primitive_poses.push_back(wall2_pose);
    wall2.operation = wall2.ADD;
    wall3.primitives.push_back(wall2p);
    wall3.primitive_poses.push_back(wall3_pose);
    wall3.operation = wall3.ADD;
    wall4.primitives.push_back(wall2p);
    wall4.primitive_poses.push_back(wall4_pose);
    wall4.operation = wall4.ADD;

    wall5.primitives.push_back(wall1p);
    wall5.primitive_poses.push_back(wall5_pose);
    wall5.operation = wall5.ADD;
    wall6.primitives.push_back(wall1p);
    wall6.primitive_poses.push_back(wall6_pose);
    wall6.operation = wall6.ADD;
    wall7.primitives.push_back(wall3p);
    wall7.primitive_poses.push_back(wall7_pose);
    wall7.operation = wall7.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    /* Shelf 1 */
    // collision_objects.push_back(shelf1);
    // collision_objects.push_back(shelf2);
    // collision_objects.push_back(shelf3);
    // collision_objects.push_back(shelf_side1);
    // collision_objects.push_back(shelf_side3);
    // collision_objects.push_back(shelf_back1);

    /* Shelf 2 */
    collision_objects.push_back(shelf4);
    collision_objects.push_back(shelf5);
    collision_objects.push_back(shelf6);
    collision_objects.push_back(shelf_back2);
    collision_objects.push_back(shelf_side2);
    collision_objects.push_back(shelf_side4);

    /* Hole in the wall */
    // collision_objects.push_back(wall1);
    // collision_objects.push_back(wall2);
    // collision_objects.push_back(wall3);
    // collision_objects.push_back(wall4);

    /* Slot in the wall */
    // collision_objects.push_back(wall5);
    // collision_objects.push_back(wall6);
    // collision_objects.push_back(wall7);

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

    auto kinematic_model = psm->getRobotModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
    std::vector<std::string> joint_names = move_group_interface.getJointNames();

    for(auto & n : joint_names){
        if(n != "panda_joint7")
        {
            acm.setEntry(n, "sb1", true);
            acm.setEntry(n, "shelf1", true);
            acm.setEntry(n, "shelf2", true);
            acm.setEntry(n, "shelf3", true);
            acm.setEntry(n, "ss1", true);
            acm.setEntry(n, "ss2", true);
        }
    }

    // acm.set 

    StateValidityChecker state_validity_checker(planning_scene, joint_model_group);
    FreedomHeuristic observation_heuristic(kinematic_state, planning_scene, joint_model_group, &state_validity_checker, acm, 2, 0.5);
    // UniformHeuristic observation_heuristic;
    // AStarHeuristic observation_heuristic;

    // Initialize MAD_RT

    // MAD_RT planner(macro_actions, names, &state_validity_checker, &observation_heuristic);
    BiMAD_RT planner(macro_actions, names, &state_validity_checker, &observation_heuristic);

    // Eigen::VectorXd start(7);
    // // start << 0.074, -1.13, -1.4, -1.08, -1.21, 1.29, 1.22;
    // // start << 0,0,0,0,0,0,0;
    // // start << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
    // // start << 0 , 0, 0, -1.343, 0, 1.413, 0.785;
    // start << -2.7 , 0, 0, -1.4, 0, 1.4, 0.785;

    // std::vector<double> start_vec(start.data(), start.data() + start.size());

    if(!state_validity_checker.CheckValidity(start))
    {
        std::cout << "Invalid Start" << std::endl;
        return 0;
    }

    // Move arm to start position
    // move_group_interface.setJointValueTarget(start_vec);
    // move_group_interface.move();


    Eigen::VectorXd goal(7);
    // goal << -1.114, -2.056, -0.3538, -1.48, 1.18, 1.678, 2.54;
    // goal << 0.07452197849980702,-1.1334201864761648,-1.4754646390263484,-1.0818732297112024,-1.2166551688908482,1.291164571285692,1.227302802458642;
    goal << 1.392 , -1.762, 1.517, -2.285, 1.727, 1.400, 0.174; // Bottom shelf
    // goal << 2.670, -0.175, -2.688, -2.217, -0.105, 2.374, -2.147; // Hole in the wall lowered
    // goal << 0, 0.401, -0.314, -2.164, -0.349, 2.025, 2.094; // alt goal
    // goal << 0.0524, 0.471, -0.995, -1.100, 0.332, 1.396, 2.880; // alt goal2
    // goal << -0.262, 0.209, 0.244, -1.466, -0.052, 1.658, 0.89; // Just through the hole
    // goal << 0.0, -0.1448, -0.052, -2.618, -0.052, 1.152, 0.873; // Before entering the hole
    // goal << -0.24, 0.70, 0, -0.87, 0 ,1.57, 0; // slot goal

    std::vector<double> goal_vec(goal.data(), goal.data() + goal.size());

    if(!state_validity_checker.CheckValidity(goal))
    {
        std::cout << "Invalid Goal" << std::endl;
        return 0;
    }

    for(int i = 0; i < 1 ; i++)
    {
        auto path = planner.plan(start, goal);
        if(!path.empty()){
            // std::cout << "Unsmoothed length: " << CalcPathLength(path) << '\n';

            ShortcutSmoother smoother(&state_validity_checker);
            smoother.Smooth(path);
            // std::cout << "Smoothed length: " << CalcPathLength(path) << '\n';
        }

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

        // move_group_interface.execute(madrt_traj);
    }


    // PrintPath(path);

    // Return arm to start position
    move_group_interface.setJointValueTarget(start_vec);
    move_group_interface.setPlanningTime(60);
    // move_group_interface.move();

    // for(int i =0; i < 25; i++)
    // {
    //     std::chrono::steady_clock::time_point tick = std::chrono::steady_clock::now();

    //     move_group_interface.setPlannerId("RRTkConfigDefault");
    //     move_group_interface.setJointValueTarget(goal_vec);
    //     moveit::planning_interface::MoveGroupInterface::Plan rrt_plan;
    //     bool success = (move_group_interface.plan(rrt_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //     std::chrono::steady_clock::time_point tock = std::chrono::steady_clock::now();
    //     std::chrono::duration<double> diff = tock - tick;
    //     double elapsed = diff.count();

    //     std::cout << '\n';
    //     std::cout << "[RRT] Found a path in " << elapsed << " seconds!" << '\n';
    // }

    // for(int i =0; i < 25; i++)
    // {
    //     std::chrono::steady_clock::time_point tick = std::chrono::steady_clock::now();

    //     move_group_interface.setPlannerId("RRTConnectkConfigDefault");
    //     move_group_interface.setJointValueTarget(goal_vec);
    //     moveit::planning_interface::MoveGroupInterface::Plan rrt_plan;
    //     bool success = (move_group_interface.plan(rrt_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //     std::chrono::steady_clock::time_point tock = std::chrono::steady_clock::now();
    //     std::chrono::duration<double> diff = tock - tick;
    //     double elapsed = diff.count();

    //     std::cout << '\n';
    //     std::cout << "[RRTConnect] Found a path in " << elapsed << " seconds!" << '\n';
    // }

}