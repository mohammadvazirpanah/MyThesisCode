#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <forsyde.hpp>
#include <math.h>
#include "global.hpp"

using namespace sc_core;
using namespace ForSyDe;
using namespace std;


void controller_func(tuple<vector<cmd_type>,vector<controller_state>>& outs,
                    const tuple<vector<odom_type>,vector<laser_type>,vector<controller_state>>& inps)
{
    auto odom_data = get<0>(inps)[0];
    auto laser_data = get<1>(inps)[0];
    auto controller_st = get<2>(inps)[0];
    switch (controller_st)
    {
        case Move_Toward:
        {
            get<0>(outs)[0] = UP;
            break;
        }
        // case :
        // {
 
        //     break;
        // }
        // case :
        // {
            
        //     break;
        // }
        // case :
        // {
            
        //     break;
        // }
        // default:
        // {

        // }
    }
    // cout<<"[Controller] command: "<<get<0>(outs)[0]<<endl;
    // sleep(1);
}

#endif