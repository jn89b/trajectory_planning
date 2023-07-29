#pragma once

/**
 * This is a dummy class to demonstrate features of the boiler plate.
 * Stole this from https://raw.githubusercontent.com/bsamseth/cpp-project/master/include/example.h
 */
class Dummy {
	public:

  /**
   * Default constructor for Dummy (does nothing).
   */
  Dummy();
  /**
   * Returns a bool.
   * @return Always True.
   */
  bool doSomething();
};


#ifdef ENABLE_DOCTEST_IN_LIBRARY
#include "doctest/doctest.h"
TEST_CASE("we can have tests in headers if we want")
{
    Dummy d;
    CHECK(d.doSomething() == true);
}
#endif
