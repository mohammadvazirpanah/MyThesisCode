

#include <iostream>
#include "report.hpp"
#include <forsyde.hpp>

using namespace sc_core;
using namespace ForSyDe;

SC_MODULE(top)
{
    SY::signal<int> srca, srcb, result;
    
    SC_CTOR(top)
    {

            #ifndef FORSYDE_COSIMULATION_WRAPPERS
            SY::make_roswrap("sub1", srca , srcb);
            #else

        //#ifndef FORSYDE_COSIMULATION_WRAPPERS
        //SY::make_roswrap("sub1", srca, srcb);
        //#endif
    }

};
