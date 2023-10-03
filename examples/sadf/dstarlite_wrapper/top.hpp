
#include <forsyde.hpp>
#include "globals.hpp"
#include "maze.hpp"
#include "controller.hpp"
#include "controller_monitor.hpp"
#include "controller_self_model.hpp"
#include "saf.hpp"

// #include <chrono>

using namespace sc_core;
using namespace ForSyDe;
using namespace std;
// using namespace std::chrono;

SC_MODULE(top)
{
   

    SC_CTOR(top)
    {
        
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

