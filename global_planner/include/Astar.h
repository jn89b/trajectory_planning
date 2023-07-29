#pragma once

class Astar{
	
    public:

    // Constructor
    Astar();

    bool doSomething();
    int add(int a, int b);

};


#ifdef ENABLE_DOCTEST_IN_LIBRARY
#include "doctest/doctest.h"
TEST_CASE("we can have tests in headers if we want")
{
    Dummy d;
    CHECK(d.doSomething() == true);
}
#endif
