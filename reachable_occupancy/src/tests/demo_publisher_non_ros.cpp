/*
This file is part of Reach-RI.

Reach-RI is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
TUM, either version 3 of the License, or
(at your option) any later version.

Reach-RI is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see https://www.gnu.org/licenses/.
*/

//#include <ros/ros.h>

#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include "reach_lib.hpp"



// read from csv data
std::vector<std::pair<std::string, std::vector<double>>> read_csv(std::string filepath) {
    std::vector<std::pair<std::string, std::vector<double>>> result;

    // input
    std::ifstream pos_data(filepath);

    // check whether file is open or not
    if (!pos_data.is_open()) {
        throw std::runtime_error("Could not open file: " + filepath);
    } else {
        std::cout << "Successfully read: " << filepath << "\n";
    }

    std::string line, colname;
    double val;

    // Read colnames
    if (pos_data.good()) {
        // Extract the first line in the file
        std::getline(pos_data, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while (std::getline(ss, colname, ',')) {
            // Initialize and add <colname, int vector> pairs to result
            result.push_back({colname, std::vector<double> {}});
        }
    }

    // Read data, line by line
    while (std::getline(pos_data, line)) {
        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Keep track of the current column index
        int colIdx = 0;

        // Extract each integer
        while (ss >> val) {
            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);

            // If the next token is a comma, ignore it and move on
            if (ss.peek() == ',') ss.ignore();

            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    pos_data.close();

    return result;
}


std::vector<reach_lib::Point> extract_frame(std::vector<std::pair<std::string,
                                            std::vector<double>>> pos_data, int frame) {
    std::vector<reach_lib::Point> pos;
    // std::vector<std::string> joints = {"RShoulder", "RForearm", "RHand", "LShoulder", "LForearm",
    // "LHand", "Neck", "Hip", "RThigh", "RShin", "RFoot", "LThigh", "LShin", "LFoot", "Head"};
    std::vector<int> index = {16, 19, 22, 43, 46, 49, 70, 1, 82, 85, 88, 100, 103, 106, 73};
    for (auto it : index) {
        reach_lib::Point p = reach_lib::Point();
        // std::cout<<"Extracting: "<<std::get<0>(pos_data[it])<<"\n";
        p.x = std::get<1>(pos_data[it])[frame];
        p.y = std::get<1>(pos_data[it+1])[frame];
        p.z = std::get<1>(pos_data[it+2])[frame];
        pos.push_back(p);
    }
    return pos;
}









int main(int argc, char** argv) {
    // ros system
    /*
    ros::init(argc, argv, "reachable_occupancy_main");
    ros::NodeHandle nh;
    ros::Rate loop_rate(60);*/


    // timer
    long double running_sum = 0.0;
    double loop_counter = 1.0;
    double average = 0.0;
    double longest = 0.0;
    int shortest = 1E9;
    bool timing = true;

    // get data
    std::string filepath = "/home/sven/catkin_ws/src/reachable_occupancy/src/tests/simple_skeleton_mvmt.csv";
    /*if (ros::param::has("/csv_path")) {
        ros::param::get("/csv_path", filepath);
    } else {
        throw "Could not get filepath from ros-parameter server!";
    }*/

    // was: "./tests/simple_skeleton_mvmt.csv"
    std::vector<std::pair<std::string, std::vector<double>>> pos_data = read_csv(filepath);

    // test parameters
    double t_a = 0.0;
    double t_b = 0.2;
    double frame_time = std::get<1>(pos_data[0])[1];
    std::vector<std::pair<int, std::vector<reach_lib::Point>>> frame_pos;
    std::vector<std::pair<int, std::vector<reach_lib::Point>>> frame_vel;
    for (int i = 0; i < pos_data.size(); i++) {
        frame_pos.push_back({i, extract_frame(pos_data, i)});
    }


    // Calculating velocity frame by frame from frame positions
    for (int i = 0; i < frame_pos.size(); i++) {
        std::vector<reach_lib::Point> pv;
        if (i == 0) {
            for (int j = 0; j < std::get<1>(pos_data[0]).size(); j++) {
                pv.push_back(reach_lib::Point());
            }
            frame_vel.push_back({i, pv});
        } else {
            std::vector<reach_lib::Point> pos_previous = std::get<1>(frame_pos[i-1]);
            std::vector<reach_lib::Point> pos_current = std::get<1>(frame_pos[i]);
            for (int j = 0; j < pos_previous.size(); j++) {
                reach_lib::Point d = pos_current[i] - pos_previous[i];
                double fact = reach_lib::Point::norm(d)/frame_time;
                d.x *= fact;
                d.y *= fact;
                d.z *= fact;
                pv.push_back(d);
            }
            frame_vel.push_back({i, pv});
        }
    }
    assert(frame_vel.size() == frame_pos.size());

    // Dummy values ACCEL and VEL ordered as defned in the joint-bodyPart map
    reach_lib::System s = reach_lib::System();

    std::map<std::string, std::pair<int, int>> index;

    // Right arm init
    index["RightUpperArm"] = std::pair<int, int> (0, 1);
    index["RightForeArm"] = std::pair<int, int> (1, 2);
    index["RightHand"] = std::pair<int, int> (2, 2);

    // Left arm init
    index["LeftUpperArm"] = std::pair<int, int> (3, 4);
    index["LeftForeArm"] = std::pair<int, int> (4, 5);
    index["LeftHand"] = std::pair<int, int> (5, 5);

    // Torso init
    index["Torso"] = std::pair<int, int> (6, 7);

    // Right leg init
    index["RightThigh"] = std::pair<int, int> (8, 9);
    index["RightShin"] = std::pair<int, int> (9, 10);
    index["RightFoot"] = std::pair<int, int> (10, 10);

    // Left leg init
    index["LeftThigh"] = std::pair<int, int> (11, 12);
    index["LeftShin"] = std::pair<int, int> (12, 13);
    index["LeftFoot"] = std::pair<int, int> (13, 13);

    // Head init
    index["Head"] = std::pair<int, int> (14, 14);

    // Maximum acceleration
    std::vector<double> max_a = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 20.0, 20.0,
                                 30.0, 30.0, 50.0, 30.0, 30.0, 50.0, 25.0};

    // Maximum velocity (the same for all joints)
    std::vector<double> max_v;
    for (int i = 0; i < 15; i++) {
        max_v.push_back(14.0);
    }

    // BodyPart thickness
    std::map<std::string, double> thickness;
    thickness["RightUpperArm"] = 0.1;
    thickness["RightForeArm"] = 0.1;
    thickness["RightHand"] = 0.205;
    thickness["LeftUpperArm"] = 0.1;
    thickness["LeftForeArm"] = 0.1;
    thickness["LeftHand"] = 0.205;
    thickness["Torso"] = 0.3;
    thickness["RightThigh"] = 0.2;
    thickness["RightShin"] = 0.2;
    thickness["RightFoot"] = 0.3;
    thickness["LeftThigh"] = 0.2;
    thickness["LeftShin"] = 0.2;
    thickness["LeftFoot"] = 0.3;
    thickness["Head"] = 0.3;


    // dummy values POS
    std::vector<double> max_v_p = {2.46, 2.46, 2.46, 2.46};
    std::vector<double> length = {0.635, 0.635, 0.81, 0.81};
    std::vector<double> thickness_p = {0.205, 0.205, 0.3, 0.3};

    std::map<std::string, std::pair<int, int>> index_p;

    // Initialize extremities
    index_p["RightArm"] = std::pair<int, int> (0, 0);  // 0
    index_p["LeftArm"] = std::pair<int, int> (1, 1);   // 3
    index_p["RightLeg"] = std::pair<int, int> (2, 2);  // 8
    index_p["LeftLeg"] = std::pair<int, int> (3, 3);   // 11

    // dummy values Pedestrian
    double arm_span = 2.0;
    double height = 2.0;
    double max_a_ped = 10.0;
    double max_v_ped = 14.0;

    // Build ACCEL and VEL human objects
    reach_lib::ArticulatedAccel human_a = reach_lib::ArticulatedAccel(s, index, thickness, max_a);
    reach_lib::ArticulatedVel human_v = reach_lib::ArticulatedVel(s, index, thickness, max_v);

    // Build POS human object
    reach_lib::ArticulatedPos human_p = reach_lib::ArticulatedPos(s, index_p, thickness_p, max_v_p, length);

    // Build Pedestrian objects
    reach_lib::PedestrianAccel pedestrian_a = reach_lib::PedestrianAccel(s, height, arm_span, max_a_ped);
    reach_lib::PedestrianVel pedestrian_v = reach_lib::PedestrianVel(s, height, arm_span, max_v_ped);

    // reach_lib::Point o = reach_lib::Point(-0.9, 0.0, 0.0);

    // Volume vol = Volume(3.5, 4.0, 3.0, 0.2, o);


    while (frame_pos.size() > 0) {
        auto t1 = std::chrono::high_resolution_clock::now();

        // current positions and velocities
        std::vector<reach_lib::Point> pos = std::get<1>(frame_pos[0]);
        std::vector<reach_lib::Point> vel = std::get<1>(frame_vel[0]);

        // Extract only the required positions from pos for ArticulatedPos
        std::vector<reach_lib::Point> pos_p;
        pos_p.push_back(pos[0]);
        pos_p.push_back(pos[3]);
        pos_p.push_back(pos[8]);
        pos_p.push_back(pos[11]);

        // Extract only the required positions and velocities from pos for Pedestrian
        std::vector<reach_lib::Point> pos_ped;
        reach_lib::Point hip_center = pos[11] - pos[8];
        hip_center.x = pos[8].x + 0.5*hip_center.x;
        hip_center.y = pos[8].y + 0.5*hip_center.y;
        hip_center.z = pos[8].z + 0.5*hip_center.z;
        pos_ped.push_back(hip_center);

        std::vector<reach_lib::Point> vel_ped;
        reach_lib::Point hip_vel = vel[11] - vel[8];
        hip_vel.x = vel[8].x + 0.5*hip_vel.x;
        hip_vel.y = vel[8].y + 0.5*hip_vel.y;
        hip_vel.z = vel[8].z + 0.5*hip_vel.z;
        vel_ped.push_back(hip_vel);

        // Update Articulated models
        human_a.update(pos, vel, t_a, t_b);
        human_v.update(pos, t_a, t_b);
        human_p.update(pos_p, t_a, t_b);

        // Update Pedestrian models
        pedestrian_a.update(pos_ped, vel_ped, t_a, t_b);
        pedestrian_v.update(pos_ped, t_a, t_b);

        std::cout << "\n" << "Got beyond update!" << "\n";

        // erase top vector element
        frame_pos.erase(frame_pos.begin());

        // loop timer
        auto t2 = std::chrono::high_resolution_clock::now();
        auto d = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        running_sum = running_sum + d;
        if (d < shortest && d > 0) {
            shortest = d;
        }
        if (d > longest) {
            longest = d;
        }
        loop_counter++;
        if (timing) {
            average = running_sum/loop_counter;
            std::cout << average << " " << longest << " " << shortest << "\n";
        }
        //ros::spinOnce();
        //loop_rate.sleep();
    }

    std::cout << "Average loop time: " << average;


    return 0;
}
