
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <forsyde.hpp>
#include <tuple>
#include <vector>
#include "globals.hpp"
//#include "maze.hpp"

using namespace std;



void controller_func(tuple<vector<cmd_type>>& outs,
                    const tuple<vector<monitor_state>>& inps)
{
    auto status = std::get<0>(inps)[0]; //status_monitor,four_directions,odom_type
    //auto controller_st = std::get<1>(inps)[0]; //controller_state
    auto status_monitor = std::get<0>(status);
    //auto four_directions = std::get<1>(status);
    auto odom_data = std::get<2>(status);
    auto current_x = odom_data[0];
    auto current_y = odom_data[1];
    auto current_z = odom_data[2];
    
    // std::cout<<"current_x = "<<current_x<<std::endl;
    // std::cout<<"current_y = "<<current_y<<std::endl;
    // std::cout<<"current_z = "<<current_z<<std::endl;


    
}

#endif
