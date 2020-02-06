#include <unit_test.h>
#include <modules/ds_custom/ds_custom_module.hpp>
#include "tests_main.h"

class DSCustomTest : public UnitTest
{
public:
virtual bool run_tests();

private:
bool test1();
};

bool DSCustomTest::run_tests()
{
    ut_run_test(test1);

    return (_tests_failed == 0);
}

bool DSCustomTest::test1()
{
    ut_compare("Bleh", true, false);

    return true;
}


ut_declare_test_c(test_ds_custom, DSCustomTest)