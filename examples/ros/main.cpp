
#include "top.hpp"

int sc_main(int argc, char **argv)
{
    top top("top");

    sc_start(12,SC_SEC);

    sc_stop();
    return 0;
}

