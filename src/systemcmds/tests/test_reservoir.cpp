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

bool test_update_regression_parameter_untrained_reservoir();
bool test_update_leakage_rate_untrained_reservoir();
bool test_update_washout_untrained_reservoir();

bool test_update_regression_parameter_trained_reservoir();
bool test_update_leakage_rate_trained_reservoir();
bool test_update_washout_trained_reservoir();
};

bool ReservoirComputerTests::run_tests()
{
    ut_run_test(test_reservoir_regression_parameter);
    ut_run_test(test_reservoir_leakage_rate);
    ut_run_test(test_reservoir_dimensions);
    ut_run_test(test_update_regression_parameter_untrained_reservoir);
    ut_run_test(test_update_leakage_rate_untrained_reservoir);
    ut_run_test(test_update_washout_untrained_reservoir);

    ut_run_test(test_update_regression_parameter_trained_reservoir);
    ut_run_test(test_update_leakage_rate_trained_reservoir);
    ut_run_test(test_update_washout_trained_reservoir);

    return (_tests_failed == 0);
}

bool ReservoirComputerTests::test_reservoir_regression_parameter()
{
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 1, 1e-06, 0);
    ut_compare_float("Check the regression parameter is set correctly by the constructor", res.get_regression_parameter(), 1e-06, 1e-07);
    return true;
}

bool ReservoirComputerTests::test_reservoir_leakage_rate()
{
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 1, 1e-06, 0);
    ut_compare_float("Check the leakage rate is set correctly by the constructor", res.get_leakage_rate(), 1, 0.001);
    return true;
}

bool ReservoirComputerTests::test_reservoir_dimensions()
{
    reservoir_computer res(1, 30, 2, 0.1, 0.8, 1, 1e-06, 0);
    ut_compare("Check the input dimension is set correctly by the constructor", res.get_input_dimension(), 1);
    ut_compare("Check the reservoir dimension is set correctly by the constructor", res.get_reservoir_dimension(), 30);
    ut_compare("Check the output dimension is set correctly by the constructor", res.get_output_dimension(), 2);
    return true;
}

bool ReservoirComputerTests::test_update_regression_parameter_untrained_reservoir()
{
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 1, 1e-06, 0);
    int return_value = res.update_regression_parameter(1);
    ut_compare_float("Check the regression parameter is set correctly by the update call", res.get_regression_parameter(), 1, 1e-07);
    ut_compare("Check the return value is 0", return_value, 0);

    return true;
}

bool ReservoirComputerTests::test_update_leakage_rate_untrained_reservoir()
{
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 2, 1e-06, 0);
    int return_value = res.update_leakage_rate(1);
    ut_compare_float("Check the leakage is set correctly by the update call", res.get_leakage_rate(), 1, 1e-07);
    ut_compare("Check the return value is 0", return_value, 0);

    return true;
}

bool ReservoirComputerTests::test_update_washout_untrained_reservoir()
{
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 2, 1e-06, 0);
    int return_value = res.update_washout(1);
    ut_compare_float("Check the washout is set correctly by the update call", res.get_washout(), 1, 1e-07);
    ut_compare("Check the return value is 0", return_value, 0);

    return true;
}

bool ReservoirComputerTests::test_update_regression_parameter_trained_reservoir()
{
    Eigen::VectorXd input(1);
    input << 4;
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 1, 1e-06, 0);
    res.train(input, input);
    int return_value = res.update_regression_parameter(1);
    ut_compare_float("Check the regression parameter is not set by the update call", res.get_regression_parameter(), 1e-06, 1e-07);
    ut_compare("Check the return value is -1", return_value, -1);

    return true;
}

bool ReservoirComputerTests::test_update_leakage_rate_trained_reservoir()
{
    Eigen::VectorXd input(1);
    input << 4;
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 1, 1e-06, 0);
    res.train(input, input);
    int return_value = res.update_leakage_rate(2);
    ut_compare_float("Check the leakage is not set by the update call", res.get_leakage_rate(), 1, 1e-07);
    ut_compare("Check the return value is -1", return_value, -1);

    return true;
}

bool ReservoirComputerTests::test_update_washout_trained_reservoir()
{
    Eigen::VectorXd input(1);
    input << 4;
    reservoir_computer res(1, 30, 1, 0.1, 0.8, 1, 1e-06, 0);
    res.train(input, input);
    int return_value = res.update_washout(2);
    ut_compare_float("Check the washout is not set by the update call", res.get_washout(), 0, 1e-07);
    ut_compare("Check the return value is -1", return_value, -1);

    return true;
}


ut_declare_test_c(test_reservoir, ReservoirComputerTests)