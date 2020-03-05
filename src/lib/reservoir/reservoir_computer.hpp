//
// Created by alpha on 2/18/20.
//

#ifndef PX4_RESERVOIR_COMPUTER_HPP
#define PX4_RESERVOIR_COMPUTER_HPP

#include <stdint.h>
#include <random>
#include <fstream>
#include <string>
#include <iostream>
//#include <drivers/drv_hrt.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/LU>
#include <iostream>

#define RETURN_CODE_DEFAULT 0
#define RETURN_CODE_ERROR -1

enum reservoir_status_t {
    NOT_TRAINED, TRAINING, TRAINED
};

class reservoir_computer {
public:
    explicit reservoir_computer(uint8_t input_vector_size, uint16_t reservoir_size, uint8_t output_vector_size,
                                double sparsity, double spectral_radius, double leakage_rate, double reg_param);

    int update_leakage_rate(double);

    int update_regression_parameter(double);

    double get_leakage_rate();

    double get_regression_parameter();

    uint8_t get_input_dimension();

    uint16_t get_reservoir_dimension();

    uint8_t get_output_dimension();

    void print_data(const Eigen::MatrixXd& input_data);

    reservoir_status_t get_reservoir_status();

    Eigen::VectorXd predict(const Eigen::RowVectorXd &input);
    void train(const Eigen::MatrixXd &input_data, const Eigen::MatrixXd &training_data);
    void reset();

private:
    //hyperparameters
    double sparsity_; // k in Canaday's paper usually around 10%
    double spectral_radius_; // rho. 1.0 is a good starting point per thesis
    double leakage_rate_; //gamma for Wendson, a for Canaday. I've seen this set to 0.3, but canaday replaced it with h/c. thesis page 20 & 21
    double regression_parameter_; //alpha. Canaday has this at 1e-6, I've seen others use 1e-8

    uint8_t input_dimension_;
    uint16_t reservoir_dimension_;
    uint8_t output_dimension_;

    reservoir_status_t current_status_; //Current state of the reservoir. Allows us to figure out whether to allow parameter updates

    Eigen::MatrixXd W_in; //Input weights
    Eigen::SparseMatrix<double> W; //Reservoir nodes in represented by a matrix
    Eigen::MatrixXd W_out; //Output weights
    Eigen::MatrixXd reservoir_evolution_;
    Eigen::RowVectorXd current_reservoir_state_;
    Eigen::VectorXd bias_;

    void update_weights(const Eigen::MatrixXd &target);

    Eigen::MatrixXd calculate_reservoir_evolution(const Eigen::MatrixXd &state_space, const Eigen::MatrixXd &input);

    void calculate_reservoir_propagation(const Eigen::MatrixXd &input, double time_step);

    void setup_reservoir();
};


#endif //PX4_RESERVOIR_COMPUTER_HPP
