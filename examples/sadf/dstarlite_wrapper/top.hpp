
#include <forsyde.hpp>
#include "forsyde/ros_hsdf_wrapper.hpp"
#include "monitor.hpp"
#include "controller.hpp"
#include "globals.hpp"
#include <cmath>
#include <limits>


using namespace sc_core;
using namespace ForSyDe;
using namespace std;




SC_MODULE(top)
{
   
    std::vector <std::string> topics_publisher=  
    {
        "/cmd_vel"
    }; 

    std::vector <std::string> topics_subscriber= 
    {
        "/odom",
        "/scan"
    };

    SDF::signal<odom_type> from_wrapper_odom;
    SDF::signal<scan_type> from_wrapper_scan;
    SDF::signal<cmd_type>  to_wrapper_cmd;
    SDF::signal<monitor_state> from_monitor_out;

    SC_CTOR(top)
    {
        SDF::make_sdf_roswrap("SdfRosWrapper", 
                            topics_publisher,
                            topics_subscriber, 
                            to_wrapper_cmd,
                            from_wrapper_odom,
                            from_wrapper_scan
                            );

        auto monitor = new SDF::combMN<tuple<monitor_state>,tuple<scan_type,odom_type>>(
        "monitor",
        monitor_func,
        {1},
        {1,1}
        );
        get<0>(monitor->iport)(from_wrapper_scan);
        get<1>(monitor->iport)(from_wrapper_odom);
        get<0>(monitor->oport)(from_monitor_out);

        auto controller = new SDF::combMN<tuple<cmd_type>,tuple<monitor_state>>(
        "controller",
        controller_func,
        {1},
        {1}
        );
        get<0>(controller->iport)(from_monitor_out);
        get<0>(controller->oport)(to_wrapper_cmd);
        
        
    }
    
#ifdef FORSYDE_INTROSPECTION
    void start_of_simulation()
    {
        ForSyDe::XMLExport dumper("gen/");
        dumper.traverse(this);
#ifdef FORSYDE_SELF_REPORTING
        while (report_pipe_fd<=0) // pipe is not open
        {
            report_pipe_fd = open("gen/self_report", O_WRONLY|O_NONBLOCK);
            if (report_pipe_fd > 0)
                report_pipe = fdopen(report_pipe_fd, "w");
            else
                cout<<"Waiting for report pipe to open..."<<endl;
        }
#endif
    }
#endif
#ifdef FORSYDE_SELF_REPORTING
    void end_of_simulation()
    {
        fclose(report_pipe);
    }
#endif

};

