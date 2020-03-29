

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
void reservoir_manager::create_resrvoirs(){
	reservoirs.push_back(new reservoir_computer(
		input_dimension_,  reservoir_dimension_, output_dimension_,
		sparsity_, spectral_radius_, leakage_rate_, regression_parameter_, washout_ ));

}

/**
 * Trains reservoirs in the list one at a time
 */
void reservoir_manager::train_reservoirs(){
	list <int> :: iterator it;
    	for(it = reservoirs.begin(); it != reservoirs.end(); ++it){
		if(*it.get_reservoir_status() == NOT_TRAINED){
			*it.train();
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
void reservoir_manager::destroy_reservoir(){
	reservoirs.pop_back();
}

/**
 *
 * Shows status of each reservoirs
 */
void reservoir_manager::show_status(){
	list <int> :: iterator it;
	int i=1;
    	for(it = reservoirs.begin(); it != reservoirs.end(); ++it){
		cout <<"Reservoir " << i <<" :" << *it.get_reservoir_status(); //PX4_INFO_RAW("")
		i++;
	}
}
