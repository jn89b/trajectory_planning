#include "Astar.h"



// Constructor class
Astar::Astar() 
{

}

bool Astar::doSomething() 
{
    // Do silly things, using some C++17 features to enforce C++17 builds only.
    return true;
}

int Astar::add(int a, int b) 
{
    return a + b;
}



#ifdef ENABLE_DOCTEST_IN_LIBRARY
#include "doctest/doctest.h"
TEST_CASE("we can have tests written here, to test impl. details")
{
    CHECK(true);
}
#endif
