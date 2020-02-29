#include "reservoir_computer.hpp"

/**
 * Constructs a new reservoir computer
 * @param input_vector_size Size of the inputs
 * @param reservoir_size Number of reservoir nodes
 * @param output_vector_size Number of outputs
 * @param sparsity How sparse to make the reservoir matrix
 * @param spectral_radius
 * @param leakage_rate Leakage rate of the nodes
 * @param reg_param Alpha parameter used to calculate output weights
 */
reservoir_computer::reservoir_computer(uint8_t input_vector_size, uint16_t reservoir_size, uint8_t output_vector_size,
double sparsity, double spectral_radius, double leakage_rate, double reg_param)
: sparsity_(sparsity), spectral_radius_(spectral_radius), leakage_rate_(leakage_rate),
regression_parameter_(reg_param), current_status_(NOT_TRAINED) {
    auto temp_matrix = Eigen::MatrixXd::Random(reservoir_size, input_vector_size);
    //input matrix reservoir_size x reservoir_size
    W_in = temp_matrix.sparseView();
    //nodes in the reservoir itself
    //Dr.Guathier suggested trying a linear topology
    W = Eigen::SparseMatrix<double>(reservoir_size, reservoir_size);
    //output weights
    W_out = Eigen::SparseMatrix<double>(output_vector_size, reservoir_size);
    setup_reservoir();
    reservoir_evolution_ = Eigen::MatrixXd(reservoir_size, 1);
}

/**
 * Calculates hyperbolic tangent for a double. We define this here because std::tanh is a template
 * function so passing a function pointer for Eigen's unaryExpr requires a function of specific type
 * @param x
 * @return hyperbolic tangent of x
 */
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
    if(current_status_ == NOT_TRAINED)
    {
        leakage_rate_ = rate;
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
    if(current_status_ == NOT_TRAINED)
    {
        regression_parameter_ = param;
    } else{
        success = RETURN_CODE_ERROR;
    }
    return success;
}

/**
 * Returns the gamma or leakage rate of the reservoir nodes
 * @return
 */
double reservoir_computer::get_leakage_rate()
{
    return leakage_rate_;
}

/**
 * Returns the regression parameter or alpha used to compute the output weights
 * @return
 */
double reservoir_computer::get_regression_parameter()
{
    return regression_parameter_;
}

/**
 * Gets the current status of the reservoir
 * @return The current state of the reservoir
 */
reservoir_status_t reservoir_computer::get_reservoir_status()
{
    return current_status_;
}

/**
 * Use runge-kutta integration to compute the new current state of the reservoir
 * @param input Input data matrix
 * @param time_step The time delta
 */
void reservoir_computer::propagate(const Eigen::MatrixXd& input, double time_step)
{
    Eigen::VectorXd current_reservoir_state = reservoir_evolution_.col(reservoir_evolution_.cols() - 1);
    //Use runge kutta estimation with the reservoir derivative function to estimate the next reservoir state
    Eigen::MatrixXd k1, k2, k3, k4;
    k1 = calculate_reservoir_evolution(current_reservoir_state, input);
    k2 = calculate_reservoir_evolution(current_reservoir_state + (time_step / 2) * k1, input);
    k3 = calculate_reservoir_evolution(current_reservoir_state + (time_step / 2) * k2, input);
    k4 = calculate_reservoir_evolution(current_reservoir_state + time_step * k3, input);
    current_reservoir_state += (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    //Add the newest reservoir state to the end of the reservoir evolution matrix
    reservoir_evolution_.conservativeResize(reservoir_evolution_.rows(), reservoir_evolution_.cols() + 1);
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
    W_out = ((X_T * reservoir_state + regression_parameter_ * I).inverse() * X_T * target).sparseView();
}

/**
 * Calculates the differential reservoir evolution
 * @param state_space The current state of the reservoir
 * @param input The input data
 * @return A vector representing
 */
Eigen::MatrixXd reservoir_computer:: calculate_reservoir_evolution(const Eigen::MatrixXd& state_space, const Eigen::MatrixXd& input)
{
    //Wendson had it as -leakage_rate_ * state_space + (leakage_rate_ * tanh(W*state_space+W_in*input)
    //might also need to add a bias vector (thesis pg. 20, 21, 22) after input
    return (1 - leakage_rate_) * state_space + leakage_rate_ * (W * state_space + W_in * input).unaryExpr(&hypertan);
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
    int percentNonZero = W.rows() * W.cols() * sparsity_;
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
    W = spectral_radius_ * W_0;
}