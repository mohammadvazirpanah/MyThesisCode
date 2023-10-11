
#include <forsyde.hpp>
#include "forsyde/ros_hsdf_wrapper.hpp"
#include <cmath>
#include <limits>


using namespace sc_core;
using namespace ForSyDe;
using namespace std;

typedef double odom_type;              
// typedef array <float,360> scan_type;  
typedef std::vector<float> scan_type;

enum cmd_type {RIGHT, UP, LEFT, DOWN, NOP};    


void filter_scan_func (std::vector<scan_type>& out1, 
                    const std::vector<scan_type>& inp1)
{
    

    // std::vector<scan_type> in_port;
    // in_port = inp1; 
    // //filter inf element

    // for (int i = 0; i < in_port.size(); i++)
    // {
    //     for (int j = 0; j < in_port[i].size(); j++)
    //     {
    //         if (in_port[i][j]!='inf')
    //         {
    //             in_port[i][j] = in_port[i][j];
    //         }
    //     }
    // }
    // // std::cout<<"in_port = "<<in_port[0][0]<<std::endl;
    // out1 = in_port;



}


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
    SDF::signal<scan_type> from_scan;
    SDF::signal<cmd_type>  to_wrapper_cmd;

    SC_CTOR(top)
    {
        SDF::make_sdf_roswrap("SdfRosWrapper", 
                            topics_publisher,
                            topics_subscriber, 
                            from_wrapper_odom,
                            from_wrapper_scan, 
                            to_wrapper_cmd
                            );

        // auto filter_function = [](scan_type& output, const scan_type& input) {
        //     filter_scan_func(output, input);
        // };

        SDF::make_comb("filter_scan", filter_scan_func, 1, 1, from_scan, from_wrapper_scan);

        auto sink2 = new SDF::sink<scan_type>("sink1",[](const scan_type& out) {cout <<"scan = " <<out << endl;});
        sink2-> iport1(from_scan);
        

        

        // auto filter_scan = new SDF::comb<scan_type>("filter_laser", [](const scan_type& in, scan_type& out) {
  
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

