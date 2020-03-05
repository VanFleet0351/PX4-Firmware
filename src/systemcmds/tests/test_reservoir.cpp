#include <unit_test.h>
#include <lib/reservoir/reservoir_computer.hpp>
#include "tests_main.h"

class ReservoirComputerTests : public UnitTest
{
public:
virtual bool run_tests();

private:
bool test_reservoir_regression_parameter();
bool test_reservoir_leakage_rate();
bool test_reservoir_dimensions();
};

bool ReservoirComputerTests::run_tests()
{
    ut_run_test(test_reservoir_regression_parameter);
    ut_run_test(test_reservoir_leakage_rate);
    ut_run_test(test_reservoir_dimensions);

    return (_tests_failed == 0);
}

bool ReservoirComputerTests::test_reservoir_regression_parameter()
{
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 1, 1e-06);
    ut_compare_float("Check the regression parameter is set correctly by the constructor", res.get_regression_parameter(), 1e-06, 1e-07);
    return true;
}

bool ReservoirComputerTests::test_reservoir_leakage_rate()
{
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 1, 1e-06);
    ut_compare_float("Check the leakage rate is set correctly by the constructor", res.get_leakage_rate(), 1, 0.001);
    return true;
}

bool ReservoirComputerTests::test_reservoir_dimensions()
{
    reservoir_computer res(1, 30, 2, 0.1, 0.8, 1, 1e-06);
    ut_compare("Check the input dimension is set correctly by the constructor", res.get_input_dimension(), 1);
    ut_compare("Check the reservoir dimension is set correctly by the constructor", res.get_reservoir_dimension(), 30);
    ut_compare("Check the output dimension is set correctly by the constructor", res.get_output_dimension(), 2);
    return true;
}

bool test_update_regression_parameter_untrained_reservoir()
{
    Eigen::VectorXd input;
    
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 1, 1e-06);
}


ut_declare_test_c(test_reservoir, ReservoirComputerTests)