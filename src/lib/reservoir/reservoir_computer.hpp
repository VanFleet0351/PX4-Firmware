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

class reservoir_computer {
public:
    explicit reservoir_computer(uint8_t input_vector_size, uint16_t reservoir_size, uint8_t output_vector_size) {
        auto temp_matrix = Eigen::MatrixXd::Random(reservoir_size, input_vector_size);
        //input matrix reservoir_size x reservoir_size
        W_in = temp_matrix.sparseView();
        //nodes in the reservoir itself
        W = Eigen::SparseMatrix<double>(reservoir_size, reservoir_size);
        //output weights
        W_out = Eigen::SparseMatrix<double>(output_vector_size, reservoir_size);
        setupReservoir();
        reservoir_evolution = Eigen::MatrixXd(reservoir_size, 1);
    }

    //Using the trained outputs, predict the correct outputs given a vector of inputs
    Eigen::VectorXd predict(const Eigen::VectorXd &input);

private:
    Eigen::SparseMatrix<double> W_in; //Input weights
    Eigen::SparseMatrix<double> W; //Reservoir nodes in represented by a matrix
    Eigen::SparseMatrix<double> W_out; //Output weights
    Eigen::MatrixXd reservoir_evolution;
    double spectralRadius; // rho
    double sparsity; // k in Canaday's paper

    Eigen::MatrixXd calculate_reservoir_evolution(const Eigen::MatrixXd& state_space, const Eigen::MatrixXd& input);
    void propagate(const Eigen::MatrixXd& input, double time_step);
    void setupReservoir();
};

Eigen::MatrixXd updateWeights(Eigen::MatrixXd reservoirState, Eigen::MatrixXd target, double gamma)
{
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(reservoirState.rows(), reservoirState.cols());
    Eigen::MatrixXd X_T = reservoirState.transpose();
    Eigen::MatrixXd demon = X_T * reservoirState * gamma * I;
    demon = demon.inverse();
    return demon * X_T * target;
}

double hypertan(double x)
{
    return std::tanh(x);
}

/*Eigen::VectorXd reservoir_computer::predict(const Eigen::VectorXd &input) {

}*/

Eigen::MatrixXd reservoir_computer:: calculate_reservoir_evolution(const Eigen::MatrixXd& state_space, const Eigen::MatrixXd& input)
{
    return state_space * (W * state_space + W_in * input).unaryExpr(&hypertan);
}

void reservoir_computer::propagate(const Eigen::MatrixXd& input, double time_step)
{
    Eigen::VectorXd current_reservoir_state = reservoir_evolution.col(reservoir_evolution.cols() - 1);
    //Use runge kutta estimation with the reservoir derivative function to estimate the next reservoir state
    Eigen::MatrixXd k1, k2, k3, k4;
    k1 = calculate_reservoir_evolution(current_reservoir_state, input);
    k2 = calculate_reservoir_evolution(current_reservoir_state + (time_step / 2) * k1, input);
    k3 = calculate_reservoir_evolution(current_reservoir_state + (time_step / 2) * k2, input);
    k4 = calculate_reservoir_evolution(current_reservoir_state + time_step * k3, input);
    current_reservoir_state += (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    //Add the newest reservoir state to the end of the reservoir evolution matrix
    reservoir_evolution.conservativeResize(reservoir_evolution.rows(), reservoir_evolution.cols() + 1);
}

//populating the network matrix
void reservoir_computer::setupReservoir() {
    std::vector<std::pair<int, int> > idx;
    std::random_device rdI;
    std::mt19937 genI(rdI());
    std::uniform_int_distribution<> disI(0, W.rows() - 1);
    std::random_device rdJ;
    std::mt19937 genJ(rdJ());
    std::uniform_int_distribution<> disJ(0, W.cols() - 1);
    std::random_device rdR;
    std::mt19937 genR(rdR());
    std::uniform_real_distribution<> disR(-1.0, 1.0);
    std::random_device rdB;
    std::mt19937 genB(rdB());
    std::uniform_real_distribution<> disB(-0.01, 0.01);

    std::vector<Eigen::Triplet<double> > tripletList;
    int percentNonZero = W.rows() * W.cols() * sparsity;   //20% on nonZero cells, from jaeger,lucosevicius, 2010
    tripletList.reserve(percentNonZero);
    for (int p = 0; p < percentNonZero; p++) {
        int i = disI(genI);
        int j = disJ(genJ);
        idx.emplace_back(i, j);
        double value = 1e-7;
        do {
            if (j < W.cols() - 1)
                value = disR(genR);
            else
                value = disB(genB);
        } while (value < 1e-6);
        tripletList.emplace_back(i, j, value);
    }
    W.setFromTriplets(tripletList.begin(), tripletList.end());
    Eigen::SparseMatrix<double> W_0(W.rows(), W.cols());
    Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double> > es(W);
    W_0 = W * (1 / fabs(es.eigenvalues()[W.rows() - 1]));//minimalESN normalize with 1.25, Jaeger 2002 with 1
    //Canaday's paper page 21
    W = spectralRadius * W_0;
}

#endif //PX4_RESERVOIR_COMPUTER_HPP
