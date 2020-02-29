//
// Created by alpha on 2/18/20.
//

#ifndef PX4_RESERVOIR_COMPUTER_HPP
#define PX4_RESERVOIR_COMPUTER_HPP

#include <stdint.h>
#include <random>
#include <drivers/drv_hrt.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

#define RETURN_CODE_DEFAULT 0
#define RETURN_CODE_ERROR -1

class reservoir_computer {
public:
    explicit reservoir_computer(uint8_t input_vector_size, uint16_t reservoir_size, uint8_t output_vector_size,
            double sparsity, double spectralRadius, double leakRate, double reg_param) {
        auto temp_matrix = Eigen::MatrixXd::Random(reservoir_size, input_vector_size);
        //input matrix reservoir_size x reservoir_size
        W_in = temp_matrix.sparseView();
        //nodes in the reservoir itself
        //Dr.Guathier suggested trying a linear topology
        W = Eigen::SparseMatrix<double>(reservoir_size, reservoir_size);
        //output weights
        W_out = Eigen::SparseMatrix<double>(output_vector_size, reservoir_size);
        this->sparsity = sparsity;
        this->spectral_radius = spectralRadius;
        this->leakage_rate = leakRate;
        this->regression_param = reg_param;
        setup_reservoir();
        reservoir_evolution = Eigen::MatrixXd(reservoir_size, 1);
        current_state = NOT_TRAINED;
    }

    enum reservoir_state_t
    {
        NOT_TRAINED, TRAINING, TRAINED
    };

    int update_leakage_rate(double);
    int update_regression_parameter(double);
    reservoir_state_t get_reservoir_state();

    //TODO
    Eigen::VectorXd predict(const Eigen::VectorXd &input);

private:
    Eigen::SparseMatrix<double> W_in; //Input weights
    Eigen::SparseMatrix<double> W; //Reservoir nodes in represented by a matrix
    Eigen::SparseMatrix<double> W_out; //Output weights
    Eigen::MatrixXd reservoir_evolution;
    //hyperparameters
    double spectral_radius; // rho. 1.0 is a good starting point per thesis
    double sparsity; // k in Canaday's paper usually around 10%
    double leakage_rate; //gamma for Wendson, a for Canaday. I've seen this set to 0.3, but canaday replaced it with h/c. thesis page 20 & 21
    double regression_param; //alpha. Canaday has this at 1e-6, I've seen others use 1e-8

    reservoir_state_t current_state; //Current state of the reservoir. Allows us to figure out whether to allow parameter updates

    void update_weights(const Eigen::MatrixXd& reservoirState, const Eigen::MatrixXd& target);
    Eigen::MatrixXd calculate_reservoir_evolution(const Eigen::MatrixXd& state_space, const Eigen::MatrixXd& input);
    void propagate(const Eigen::MatrixXd& input, double time_step);
    void setup_reservoir();
};




#endif //PX4_RESERVOIR_COMPUTER_HPP
