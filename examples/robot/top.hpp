#include <forsyde.hpp>
#include "forsyde/ros_wrapper.hpp"
#include <iostream>
#include <controller.hpp>
#include <global.hpp>
// #include <controller_transform.hpp>
#include "sensor_msgs/Range.h"
#include <math.h>
#include <limits.h>

using namespace sc_core;
using namespace ForSyDe;
using namespace std; 




void source_func(double& out1, const double& inp1)
{
    out1 = inp1;
    std::cout << "input value: " << out1 << std::endl;  

}

void report_func(double inp1)
{
    std::cout << "output value: " << inp1 << std::endl;
}



SC_MODULE(top)
{

    SDF::signal<double> from_wrapper1;
    SDF::signal<double> from_wrapper2;
    SDF::signal<double> to_wrapper1;

    SDF::signal<odom_type> odom_out;
    SDF::signal<laser_type> laser_out;
    SDF::signal<cmd_type> controller_cmd_out;
    SDF::signal<controller_state> controller_state_out, controller_state_inp;

    std::vector <std::string> topics_publisher=  
    {
        "/cmd_vel"
    }; 

    std::vector <std::string> topics_subscriber= 
    {
        "/odom",
        "/scan"
    };

    SC_CTOR(top)
    {

        SDF::make_source("ToRos", source_func, 1.0, 10.0, to_wrapper1);

        SDF::make_roswrap("roswrap", topics_publisher,
                        topics_subscriber, 
                        from_wrapper1,
                        to_wrapper1
                        );

        auto controller = new SDF::combMN<tuple<cmd_type,controller_state>,tuple<odom_type,laser_type,controller_state>>(
        "controller",
        controller_func,
        {1,1},
        {1,1,1}
        );

        get<0>(controller->iport)(odom_out);
        get<1>(controller->iport)(laser_out);
        get<2>(controller->iport)(controller_state_inp);
        get<0>(controller->oport)(controller_cmd_out);
        get<1>(controller->oport)(controller_state_out);

        SDF::make_sink("report1", report_func, from_wrapper1);





    }

    #ifdef FORSYDE_INTROSPECTION
        void start_of_simulation()
        {
            ForSyDe::XMLExport dumper("gen/");
            dumper.traverse(this);
        }
    #endif
};

