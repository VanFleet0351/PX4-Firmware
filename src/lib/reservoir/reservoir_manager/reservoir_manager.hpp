#ifndef PX4_RESERVOIR_MANAGER_HPP
#define PX4_RESERVOIR_MANAGER_HPP

#include "reservoir/reservoir_computer.hpp"
#include <iostream>
#include <list>
#include <iterator>
#include <platforms/px4_log.h>

class reservoir_manager{
	public:
	explicit reservoir_manager(uint8_t input_vector_size, uint16_t reservoir_size,
                                uint8_t output_vector_size, double sparsity, double spectral_radius,
                                double leakage_rate, double reg_param, double washout);


	int add_reservoir();
    int add_reservoir(uint16_t reservoir_dimension);

	void destroy_all_reservoirs();
	void destroy_last_reservoir();
    Eigen::RowVectorXd predict(const Eigen::RowVectorXd &input_data);
	void train_last_reservoir(const Eigen::MatrixXd &input_data, const Eigen::MatrixXd &training_data);
    void print_reservoirs_info();

    void update_leakage_rate(double);
    void update_regression_parameter(double alpha);
    void update_washout(double washout);



	private:

    std::list<reservoir_computer> reservoirs_;

    //hyperparameters
	double sparsity_; // k in Canaday's paper usually around 10%
	double spectral_radius_; // rho. 1.0 is a good starting point per thesis
	double leakage_rate_; //gamma for Wendson, a for Canaday. I've seen this set to 0.3, but canaday replaced it with h/c. thesis page 20 & 21
	double regression_parameter_; //alpha. Canaday has this at 1e-6, I've seen others use 1e-8
	double washout_;

	uint8_t input_dimension_;
	uint16_t reservoir_dimension_;
	uint8_t output_dimension_;
};
#endif
