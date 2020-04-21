#include "reservoir_manager.hpp"


reservoir_manager::reservoir_manager(uint8_t input_vector_size, uint16_t reservoir_size,
                                     uint8_t output_vector_size, double sparsity, double spectral_radius,
                                     double leakage_rate, double reg_param, double washout)
        : sparsity_(sparsity), spectral_radius_(spectral_radius), leakage_rate_(leakage_rate),
          regression_parameter_(reg_param), washout_(washout), input_dimension_(input_vector_size),
          reservoir_dimension_(reservoir_size), output_dimension_(output_vector_size) {

}

/**
 * Creates a reservoir of default sizes and add it to the list
 */
void reservoir_manager::add_reservoir() {
    add_reservoir(reservoir_dimension_);
}

/**
 * Create reservoir of dimneions reservoir_dimension
 * @param reservoir_dimension
 */
void reservoir_manager::add_reservoir(uint16_t reservoir_dimension) {
    reservoirs_.emplace_back(
            input_dimension_, reservoir_dimension, output_dimension_,
            sparsity_, spectral_radius_, leakage_rate_, regression_parameter_, washout_);
}

/**
 * Trains reservoirs in the list one at a time
 */
void reservoir_manager::train_last_reservoir(const Eigen::MatrixXd &input_data, const Eigen::MatrixXd &training_data) {
    reservoirs_.back().train(input_data, training_data);
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
        std::cout << i << "\t" << res.get_reservoir_dimension() << "\t" << res.get_reservoir_status() << "\t"
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
Eigen::VectorXd reservoir_manager::predict(const Eigen::RowVectorXd &input_data) {
    Eigen::VectorXd result = Eigen::VectorXd::Zero(output_dimension_);
    for (reservoir_computer &res: reservoirs_) {
        if (res.get_reservoir_status() == TRAINED) {
            result += res.predict(input_data);
        }
    }
    return result;
}