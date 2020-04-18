#include "reservoir_manager.hpp"


reservoir_manager::reservoir_manager(uint8_t input_vector_size, uint16_t reservoir_size,
                                       uint8_t output_vector_size, double sparsity, double spectral_radius,
                                       double leakage_rate, double reg_param, double washout)
        : sparsity_(sparsity), spectral_radius_(spectral_radius), leakage_rate_(leakage_rate),
          regression_parameter_(reg_param), washout_(washout),  input_dimension_(input_vector_size),
          reservoir_dimension_(reservoir_size), output_dimension_(output_vector_size){

	  }
/**
 * Creates a reservoir and adds it to the list
 */
void reservoir_manager::create_reservoir(){
	reservoirs.emplace_back(
		input_dimension_,  reservoir_dimension_, output_dimension_,
		sparsity_, spectral_radius_, leakage_rate_, regression_parameter_, washout_);

}

/**
 * Trains reservoirs in the list one at a time
 */
void reservoir_manager::train_reservoirs(){
    for(reservoir_computer &res: reservoirs){
		if(res.get_reservoir_status() == NOT_TRAINED){
            //res.train();
		}
	}
}

/**
 * Destroys all the reservoirs in the list. clears the list
 *
 */
void reservoir_manager::destroy_reservoirs(){
	reservoirs.clear();
}

/**
 * Deletes one reservoir from the list
 */
void reservoir_manager::destroy_last_reservoir(){
	reservoirs.pop_back();
}

void reservoir_manager::update_regression_parameter(double alpha) {
    // res.update_regression_parameter(alpha);
}


/**
 *
 * Shows status of each reservoirs
 */
void reservoir_manager::show_status(){
	int i=1;
    for(reservoir_computer &res: reservoirs){
		std::cout <<"Reservoir " << i <<" :" << res.get_reservoir_status(); //PX4_INFO_RAW("")
		i++;
	}
}

/**
 * Get prediction from the trained reservoirs.
 * @param input_data
 * @return Eigen vector of size output_dimension_
 */
Eigen::VectorXd reservoir_manager::predict(const Eigen::RowVectorXd &input_data)
{
    Eigen::VectorXd result = Eigen::VectorXd::Zero(output_dimension_);
    for(reservoir_computer &res: reservoirs)
    {
        if(res.get_reservoir_status() == TRAINED)
        {
            result += res.predict(input_data);
        }
    }
    return result;
}