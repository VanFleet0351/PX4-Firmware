#include "reservoir_manager.hpp"

/**
 * Constructs a new reservoir manager
 * @param input_vector_size Size of the inputs
 * @param reservoir_size Number of reservoir nodes
 * @param output_vector_size Number of outputs
 * @param sparsity How sparse to make the reservoir matrix
 * @param spectral_radius
 * @param leakage_rate Leakage rate of the nodes
 * @param reg_param Alpha parameter used to calculate output weights
 * @param washout the percentage of reservoir states to dump so we avoid garbage data
 */
reservoir_manager::reservoir_manager(uint8_t input_vector_size, uint16_t reservoir_size,
                                     uint8_t output_vector_size, double sparsity, double spectral_radius,
                                     double leakage_rate, double reg_param, double washout)
        : sparsity_(sparsity), spectral_radius_(spectral_radius), leakage_rate_(leakage_rate),
          regression_parameter_(reg_param), washout_(washout), input_dimension_(input_vector_size),
          reservoir_dimension_(reservoir_size), output_dimension_(output_vector_size) {

}

/**
 * Attempt to add a default dimensioned reservoir to the manager
 * @return 0 on success, -1 on failure
 */
int reservoir_manager::add_reservoir() {
    return add_reservoir(reservoir_dimension_);
}

/**
 * Attempt to add a reservoir of dimension reservoir_dimension. If there is an untrained reservoir
 * at the top of the stack we do not add a new reservoir
 * @param reservoir_dimension
 * @return 0 on success, -1 on error
 */
int reservoir_manager::add_reservoir(uint16_t reservoir_dimension) {
    int success = 0;
    if (!reservoirs_.empty()) {
        if (reservoirs_.back().get_status() == TRAINED) {
            reservoirs_.emplace_back(
                    input_dimension_, reservoir_dimension, output_dimension_,
                    sparsity_, spectral_radius_, leakage_rate_, regression_parameter_, washout_);
        } else { success = -1; }
    } else {
        reservoirs_.emplace_back(
                input_dimension_, reservoir_dimension, output_dimension_,
                sparsity_, spectral_radius_, leakage_rate_, regression_parameter_, washout_);
    }
    return success;
}

/**
 * Attempt to train the last reservoir
 * @param input_data the input values to the reservoir
 * @param training_data the training (output) values to the reservoir
 */
void reservoir_manager::train_last_reservoir(const Eigen::MatrixXd &input_data, const Eigen::MatrixXd &training_data) {
    if (!reservoirs_.empty()) {
        if(reservoirs_.back().get_status() == NOT_TRAINED)
        {
            reservoirs_.back().train(input_data, training_data);
        }
    } else {

    }
}

/**
 * Destroys all the reservoirs in the list. clears the list
 *
 */
void reservoir_manager::destroy_all_reservoirs() {
    reservoirs_.clear();
}

/**
 * Deletes one reservoir from the list
 */
void reservoir_manager::destroy_last_reservoir() {
    reservoirs_.pop_back();
}

/**
 * Updates the leakage parameter for the next added reservoir
 * @param leakage
 */
void reservoir_manager::update_leakage_rate(double leakage) {
    leakage_rate_ = leakage;
}

/**
 * Updates the regression parameter for the next added reservoid
 * @param alpha
 */
void reservoir_manager::update_regression_parameter(double alpha) {
    regression_parameter_ = alpha;
}


/**
 * Updates the washout parameter for the next added reservoir
 * @param washout
 */
void reservoir_manager::update_washout(double washout) {
    washout_ = washout;
}

/**
 * Prints the info for all the reservoirs in the maanger
 */
void reservoir_manager::print_reservoirs_info() {
    int i = 1;
    std::cout << "Res\tDim\tStatus\tAlpha\tLeakage\tWashout" << std::endl;
    for (reservoir_computer &res: reservoirs_) {
        std::cout << i << "\t" << res.get_reservoir_dimension() << "\t" << res.get_status() << "\t"
                  << res.get_regression_parameter() << "\t" << res.get_leakage_rate() << "\t" << res.get_washout()
                  << std::endl;
        i++;
    }
}


/**
 * Get prediction from the trained reservoirs.
 * @param input_data
 * @return Eigen vector of size output_dimension_
 */
Eigen::RowVectorXd reservoir_manager::predict(const Eigen::RowVectorXd &input_data) {
    Eigen::VectorXd result = Eigen::VectorXd::Zero(output_dimension_);
    for (reservoir_computer &res: reservoirs_) {
        if (res.get_status() == TRAINED) {
            result += res.predict(input_data);
        }
    }
    return result;
}