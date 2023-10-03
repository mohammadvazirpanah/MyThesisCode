#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <forsyde.hpp>
#include <tuple>
#include <array>
#include <math.h>
#define pi 3.14159265359
using namespace ForSyDe;
using namespace std;


enum controller_state {Angle_Changer, 
                 Move_Toward, 
                 Obstacle_Avoid, 
                 Move_Far, 
                 Move_Near};

typedef tuple<int,int> odom_type;               // (x,y)
typedef array<bool,4> laser_type;               // (right,up,left,down)
enum cmd_type {RIGHT, UP, LEFT, DOWN, NOP};     // move command and direction


#endif
