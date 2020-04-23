#include "reservoir_computer.hpp"

/**
 * Calculates hyperbolic tangent for a double. We define this here because std::tanh is a template
 * function so passing a function pointer for Eigen's unaryExpr requires a function of specific type
 * @param x
 * @return hyperbolic tangent of x
 */
inline double hypertan(double x) {
    return std::tanh(x);
}

/**
 * Calculate the absoulute value of a double. We define this here because std::fabs is a template
 * function so passing a function pointer for Eigen's unaryExpr requires a function of specific type
 * @param x
 * @return absolute value of x
 */
inline double absolute_double(double x) {
    return std::fabs(x);
}

/**
 * Constructs a new reservoir computer
 * @param input_vector_size Size of the inputs
 * @param reservoir_size Number of reservoir nodes
 * @param output_vector_size Number of outputs
 * @param sparsity How sparse to make the reservoir matrix
 * @param spectral_radius
 * @param leakage_rate Leakage rate of the nodes
 * @param reg_param Alpha parameter used to calculate output weights
 * @param washout the percentage of reservoir states to dump so we avoid garbage data
 */
reservoir_computer::reservoir_computer(uint8_t input_vector_size, uint16_t reservoir_size,
                                       uint8_t output_vector_size, double sparsity, double spectral_radius,
                                       double leakage_rate, double reg_param, double washout)
        : sparsity_(sparsity), spectral_radius_(spectral_radius), leakage_rate_(leakage_rate),
          regression_parameter_(reg_param), washout_(washout), input_dimension_(input_vector_size),
          reservoir_dimension_(reservoir_size), output_dimension_(output_vector_size), current_status_(NOT_TRAINED) {
    //input matrix reservoir_size x reservoir_size
    W_in = Eigen::MatrixXd::Random(input_dimension_, reservoir_dimension_);
    //nodes in the reservoir itself
    //Dr.Guathier suggested trying a linear topology
    W = Eigen::SparseMatrix<double>(reservoir_dimension_, reservoir_dimension_);
    //output weights
    W_out = Eigen::MatrixXd(reservoir_dimension_, output_dimension_);
    setup_reservoir();
    reservoir_evolution_ = Eigen::MatrixXd::Constant(1, reservoir_dimension_, 0);
    bias_ = Eigen::RowVectorXd::Random(reservoir_dimension_);
}

/**
 * Initialize the reservoir. Generates the sparse matrix of size reservoir_dimension_ * reservoir_dimension_
 * This also normalizes the reservoir matrix based on the eigen value
 */
void reservoir_computer::setup_reservoir() {
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

    std::vector<Eigen::Triplet<double> > tripletList;
    int nonzero_node_count = sparsity_ * (W.rows() * W.cols());
    tripletList.reserve(nonzero_node_count);
    for (int p = 0; p < nonzero_node_count; p++) {
        int i = disI(genI);
        int j = disJ(genJ);
        double value = disR(genR);
        tripletList.emplace_back(i, j, value);
    }
    W.setFromTriplets(tripletList.begin(), tripletList.end());
    Eigen::SparseMatrix<double> W_0(W.rows(), W.cols());
    Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double> > es(W);
    double max_eigen_value = es.eigenvalues().unaryExpr(&absolute_double).maxCoeff();
    W_0 = W * (1 / absolute_double(es.eigenvalues()[W.rows() - 1]));//minimalESN normalize with 1.25, Jaeger 2002 with 1
    //thesis pg. 21
    W = spectral_radius_ / max_eigen_value * W;
}

/**
 * Train the reservoir
 * @param input_data matrix representing the input data or x.
 * Note on the dimenionality. The number of rows corresponds to the number of datapoints.
 * The column dimension is the dimension of the inputs.
 * @param training_data matrix representing the output data or f(x).
 * Note on the dimenionality. The number of rows corresponds to the number of datapoints.
 * The column dimension is the dimension of the outputs.
 * @return success
 */
int reservoir_computer::train(const Eigen::MatrixXd &input_data, const Eigen::MatrixXd &training_data) {
    printf("In row %d Out row %d In col %d Out col %d\n", input_data.rows(), training_data.rows(), input_data.cols(), training_data.cols());
    if(input_data.rows() == training_data.rows() && input_data.cols() == input_dimension_ && training_data.cols() == output_dimension_)
    {
        current_status_ = TRAINING;

        int discard_amount = input_data.rows() * washout_;
        current_reservoir_state_ = reservoir_evolution_.row(reservoir_evolution_.rows() - 1);
        for (long i = 0; i < input_data.rows(); i++) {
            //Get the current state of the reservoir
            //Calculate the propogation of the reservoir
            current_reservoir_state_ = calculate_reservoir_propagation(input_data.row(i), current_reservoir_state_, 1);
            //Expand the reservoir evolution matrix by 1 and set the new value
            if (i >= discard_amount) {
                reservoir_evolution_.conservativeResize(reservoir_evolution_.rows() + 1, reservoir_evolution_.cols());
                reservoir_evolution_.row(reservoir_evolution_.rows() - 1) = current_reservoir_state_;
            }
        }
        update_weights(training_data.bottomRows(training_data.rows() - discard_amount));

        current_status_ = TRAINED;
        return RETURN_CODE_DEFAULT;
    }
    return RETURN_CODE_ERROR;
}

/**
 * Use runge-kutta integration to compute the new current state of the reservoir
 * @param input Input data matrix
 * @param time_step The time delta
 */
Eigen::RowVectorXd reservoir_computer::calculate_reservoir_propagation(const Eigen::RowVectorXd &input,
                                                                       const Eigen::RowVectorXd &previous_state,
                                                                       double time_step) {
    //Use runge kutta estimation with the reservoir derivative function to estimate the next reservoir state
    Eigen::MatrixXd k1, k2, k3, k4;
    k1 = calculate_reservoir_evolution(previous_state, input);
    k2 = calculate_reservoir_evolution(previous_state + (time_step / 2.0) * k1, input);
    k3 = calculate_reservoir_evolution(previous_state + (time_step / 2.0) * k2, input);
    k4 = calculate_reservoir_evolution(previous_state + time_step * k3, input);
    return previous_state + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
}

/**
 * Calculates the differential reservoir evolution
 * @param state_space The current state of the reservoir
 * @param input The input data
 * @return A vector representing
 */
Eigen::RowVectorXd
reservoir_computer::calculate_reservoir_evolution(const Eigen::RowVectorXd &state_space,
                                                  const Eigen::RowVectorXd &input) {
#ifdef BLEH
    std::cout << "Input size: " << input.rows() << "x" << input.cols() << std::endl;
    std::cout << "W_in: " << W_in.rows() << "x" << W_in.cols() << std::endl;
    std::cout << "W_in * Input: " << (input * W_in).rows() << "x" << (input * W_in).cols() << std::endl;
#endif
    //Wendson had it as -leakage_rate_ * state_space + (leakage_rate_ * tanh(W*state_space+W_in*input)
    //might also need to add a bias vector (thesis pg. 20, 21, 22) after input
    return ((-leakage_rate_) * state_space) +
           leakage_rate_ * (state_space * W + input * W_in + bias_).unaryExpr(&hypertan);
}

/**
 * Update the weights of the W_out matrix
 * @param target
 */
void reservoir_computer::update_weights(const Eigen::MatrixXd &target) {
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(W.rows(), W.cols());
    Eigen::MatrixXd X = reservoir_evolution_.bottomRows(reservoir_evolution_.rows() - 1);
    Eigen::MatrixXd X_T = X.transpose();
    //output matrix weights

    W_out = (X_T * X + regression_parameter_ * I).inverse() * X_T * target;

#ifdef RESERVOIR_DEBUG
    std::cout << "X size: " << X.rows() << "x" << X.cols() << std::endl;
    std::cout << "X_T size: " << X_T.rows() << "x" << X_T.cols() << std::endl;
    std::cout << "target size: " << target.rows() << "x" << target.cols() << std::endl;
    std::cout << "W_out: " << W_out.rows() << "x" << W_out.cols() << std::endl;
#endif
}

/**
 * Predict a new value from the reservoir
 * @param input Row vector of input data to calculate an output value
 * @return a row vector representing the outptut data
 */
Eigen::RowVectorXd reservoir_computer::predict(const Eigen::RowVectorXd &input) {
    current_reservoir_state_ = calculate_reservoir_propagation(input, current_reservoir_state_, 1);
    return current_reservoir_state_ * W_out;
}

/**
 * Reset the reservoir.
 */
void reservoir_computer::reset() {
    W_in = Eigen::MatrixXd::Random(input_dimension_, reservoir_dimension_);
    //nodes in the reservoir itself
    W = Eigen::SparseMatrix<double>(reservoir_dimension_, reservoir_dimension_);
    //output weights
    W_out = Eigen::MatrixXd(reservoir_dimension_, output_dimension_);
    setup_reservoir();
    reservoir_evolution_ = Eigen::MatrixXd::Constant(1, reservoir_dimension_, 0);
    bias_ = Eigen::RowVectorXd::Random(reservoir_dimension_);
    current_status_ = NOT_TRAINED;
}

/**
 * Used to print output data to file for plotting and testing. A python companion script was developed to
 * plot the output data and should be packaged with the other resources for this project.
 *
 * @param input_data
 */
void reservoir_computer::print_data(const Eigen::MatrixXd &input_data) {
    std::ofstream writer("data.csv");
    if (!writer) {
        return;
    }
    Eigen::VectorXd output_data;
    for (int i = 0; i < input_data.rows(); i++) {
        writer << input_data.row(i) << ", " << predict(input_data.row(i)) << std::endl;
    }
    writer.close();
    std::cout << "Finished writing" << std::endl;
}

/*
 * Parameter updates
 */
/**
 * Update the leakage rate when computing the reservoir evolution. If the reservoir is currently training or trained, it will not update.
 * @param rate The new leakage rate
 * @return 0 for success, -1 for error
 */
int reservoir_computer::update_leakage_rate(double rate) {
    int success = RETURN_CODE_DEFAULT;
    if (current_status_ == NOT_TRAINED) {
        leakage_rate_ = rate;
    } else {
        success = RETURN_CODE_ERROR;
    }
    return success;
}

/**
 * Update the ridge regression alpha parameter. If the reservoir is currently trained or training it will not update.
 * @param param The new alpha parameter
 * @return 0 for success, -1 for error
 */
int reservoir_computer::update_regression_parameter(double param) {
    int success = RETURN_CODE_DEFAULT;
    if (current_status_ == NOT_TRAINED) {
        regression_parameter_ = param;
    } else {
        success = RETURN_CODE_ERROR;
    }
    return success;
}

/**
 * Update the washout parameter. If the reservoir is currently trained or training it will not update.
 * @param param The new washout parameter
 * @return 0 for success, -1 for error
 */
int reservoir_computer::update_washout(double washout) {
    int success = RETURN_CODE_DEFAULT;
    if (current_status_ == NOT_TRAINED) {
        washout_ = washout;
    } else {
        success = RETURN_CODE_ERROR;
    }
    return success;
}

/**
 * Returns the gamma or leakage rate of the reservoir nodes
 * @return
 */
double reservoir_computer::get_leakage_rate() {
    return leakage_rate_;
}

/**
 * Returns the regression parameter or alpha used to compute the output weights
 * @return
 */
double reservoir_computer::get_regression_parameter() {
    return regression_parameter_;
}

/**
 * Returns the washout parameter or percentage of reservoir states we throw away
 * @return
 */
double reservoir_computer::get_washout() {
    return washout_;
}


/**
 * Gets the current status of the reservoir
 * @return The current state of the reservoir
 */
reservoir_status_t reservoir_computer::get_status() {
    return current_status_;
}

/**
 * Gets the input size of the reservoir
 * @return
 */
uint8_t reservoir_computer::get_input_dimension() {
    return input_dimension_;
}

/**
 * Gets the size of the reservoir
 * @return
 */
uint16_t reservoir_computer::get_reservoir_dimension() {
    return reservoir_dimension_;
}

/**
 * Gets the output size of the reservoir
 * @return
 */
uint8_t reservoir_computer::get_output_dimension() {
    return output_dimension_;
}
