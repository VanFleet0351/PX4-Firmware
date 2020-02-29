#include "reservoir_computer.hpp"

inline double hypertan(double x)
{
    return std::tanh(x);
}

/*
 * Parameter updates
 */
/**
 * Update the leakage rate when computing the reservoir evolution. If the reservoir is currently training or trained, it will not update.
 * @param rate The new leakage rate
 * @return 0 for success, -1 for error
 */
int reservoir_computer::update_leakage_rate(double rate)
{
    int success = RETURN_CODE_DEFAULT;
    if(current_state == NOT_TRAINED)
    {
        leakage_rate = rate;
    } else{
        success = RETURN_CODE_ERROR;
    }
    return success;
}

/**
 * Update the ridge regression alpha parameter. If the reservoir is currently trained or training it will not update.
 * @param param The new alpha parameter
 * @return 0 for success, -1 for error
 */
int reservoir_computer::update_regression_parameter(double param)
{
    int success = RETURN_CODE_DEFAULT;
    if(current_state == NOT_TRAINED)
    {
        regression_param = param;
    } else{
        success = RETURN_CODE_ERROR;
    }
    return success;
}

/**
 * Gets the current status of the reservoir
 * @return The current state of the reservoir
 */
reservoir_computer::reservoir_state_t reservoir_computer::get_reservoir_state()
{
    return current_state;
}

/**
 * Use runge-kutta integration to compute the new current state of the reservoir
 * @param input Input data matrix
 * @param time_step The time delta
 */
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

/**
 * Update the weights of the W_out matrix
 * @param reservoir_state
 * @param target
 */
void reservoir_computer::update_weights(const Eigen::MatrixXd& reservoir_state, const Eigen::MatrixXd& target)
{
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(reservoir_state.rows(), reservoir_state.cols());
    Eigen::MatrixXd X_T = reservoir_state.transpose();
    //output matrix weights
    W_out = ((X_T * reservoir_state + regression_param * I).inverse() * X_T * target).sparseView();
}

/**
 * Calculates the differential reservoir evolution
 * @param state_space The current state of the reservoir
 * @param input The input data
 * @return A vector representing
 */
Eigen::MatrixXd reservoir_computer:: calculate_reservoir_evolution(const Eigen::MatrixXd& state_space, const Eigen::MatrixXd& input)
{
    //Wendson had it as -leakage_rate * state_space + (leakage_rate * tanh(W*state_space+W_in*input)
    //might also need to add a bias vector (thesis pg. 20, 21, 22) after input
    return (1 - leakage_rate) * state_space + leakage_rate * (W * state_space + W_in * input).unaryExpr(&hypertan);
}

/*
 * Initialization
 */
void reservoir_computer::setup_reservoir() {
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
    int percentNonZero = W.rows() * W.cols() * sparsity;
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
    //thesis pg. 21
    W = spectral_radius * W_0;
}