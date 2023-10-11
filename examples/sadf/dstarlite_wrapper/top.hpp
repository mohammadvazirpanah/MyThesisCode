
#include <forsyde.hpp>
#include "forsyde/ros_hsdf_wrapper.hpp"


using namespace sc_core;
using namespace ForSyDe;
using namespace std;

typedef double odom_type;              
typedef array <float,360> scan_type;  

enum cmd_type {RIGHT, UP, LEFT, DOWN, NOP};     

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

    SDF::signal<odom_type> from_wrapper1;
    SDF::signal<scan_type> from_wrapper2;
    SDF::signal<cmd_type>  to_wrapper1;

    SC_CTOR(top)
    {
        SDF::make_sdf_roswrap("SdfRosWrapper", 
                            topics_publisher,
                            topics_subscriber, 
                            from_wrapper1,
                            from_wrapper2, 
                            to_wrapper1
                            );

        auto sink2 = new SDF::sink<scan_type>("sink1",[](const scan_type& out) {cout <<"scan = " <<out << endl;});
        sink2-> iport1(from_wrapper2);
        
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

