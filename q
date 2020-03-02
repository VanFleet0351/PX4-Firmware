[1mdiff --git a/src/lib/reservoir/reservoir_computer.cpp b/src/lib/reservoir/reservoir_computer.cpp[m
[1mindex dda6f19055..4cb67e6795 100644[m
[1m--- a/src/lib/reservoir/reservoir_computer.cpp[m
[1m+++ b/src/lib/reservoir/reservoir_computer.cpp[m
[36m@@ -1,7 +1,12 @@[m
 #include "reservoir_computer.hpp"[m
 [m
[31m-inline double hypertan(double x)[m
[31m-{[m
[32m+[m[32m/**[m
[32m+[m[32m * Calculates hyperbolic tangent for a double. We define this here because std::tanh is a template[m
[32m+[m[32m * function so passing a function pointer for Eigen's unaryExpr requires a function of specific type[m
[32m+[m[32m * @param x[m
[32m+[m[32m * @return hyperbolic tangent of x[m
[32m+[m[32m */[m
[32m+[m[32minline double hypertan(double x) {[m
     return std::tanh(x);[m
 }[m
 [m
[36m@@ -9,17 +14,16 @@[m [minline double hypertan(double x)[m
  * Parameter updates[m
  */[m
 /**[m
[31m- * Update the leakage rate when computing the reservoir evolution. If the reservoir is currently training or trained, it will not update.[m
[32m+[m[32m * Update the leakage rate when computing the reservoir evolution. If the reservoir is currently training or[m
[32m+[m[32m * trained, it will not update.[m
  * @param rate The new leakage rate[m
  * @return 0 for success, -1 for error[m
  */[m
[31m-int reservoir_computer::update_leakage_rate(double rate)[m
[31m-{[m
[32m+[m[32mint reservoir_computer::update_leakage_rate(double rate) {[m
     int success = RETURN_CODE_DEFAULT;[m
[31m-    if(current_status == NOT_TRAINED)[m
[31m-    {[m
[32m+[m[32m    if (current_status == NOT_TRAINED) {[m
         leakage_rate = rate;[m
[31m-    } else{[m
[32m+[m[32m    } else {[m
         success = RETURN_CODE_ERROR;[m
     }[m
     return success;[m
[36m@@ -30,13 +34,11 @@[m [mint reservoir_computer::update_leakage_rate(double rate)[m
  * @param param The new alpha parameter[m
  * @return 0 for success, -1 for error[m
  */[m
[31m-int reservoir_computer::update_regression_parameter(double param)[m
[31m-{[m
[32m+[m[32mint reservoir_computer::update_regression_parameter(double param) {[m
     int success = RETURN_CODE_DEFAULT;[m
[31m-    if(current_status == NOT_TRAINED)[m
[31m-    {[m
[32m+[m[32m    if (current_status == NOT_TRAINED) {[m
         regression_param = param;[m
[31m-    } else{[m
[32m+[m[32m    } else {[m
         success = RETURN_CODE_ERROR;[m
     }[m
     return success;[m
[36m@@ -46,8 +48,7 @@[m [mint reservoir_computer::update_regression_parameter(double param)[m
  * Gets the current status of the reservoir[m
  * @return The current state of the reservoir[m
  */[m
[31m-reservoir_status_t reservoir_computer::get_reservoir_status()[m
[31m-{[m
[32m+[m[32mreservoir_status_t reservoir_computer::get_reservoir_status() {[m
     return current_status;[m
 }[m
 [m
[36m@@ -56,8 +57,7 @@[m [mreservoir_status_t reservoir_computer::get_reservoir_status()[m
  * @param input Input data matrix[m
  * @param time_step The time delta[m
  */[m
[31m-void reservoir_computer::propagate(const Eigen::MatrixXd& input, double time_step)[m
[31m-{[m
[32m+[m[32mvoid reservoir_computer::propagate(const Eigen::MatrixXd &input, double time_step) {[m
     Eigen::VectorXd current_reservoir_state = reservoir_evolution.col(reservoir_evolution.cols() - 1);[m
     //Use runge kutta estimation with the reservoir derivative function to estimate the next reservoir state[m
     Eigen::MatrixXd k1, k2, k3, k4;[m
[36m@@ -75,8 +75,7 @@[m [mvoid reservoir_computer::propagate(const Eigen::MatrixXd& input, double time_ste[m
  * @param reservoir_state[m
  * @param target[m
  */[m
[31m-void reservoir_computer::update_weights(const Eigen::MatrixXd& reservoir_state, const Eigen::MatrixXd& target)[m
[31m-{[m
[32m+[m[32mvoid reservoir_computer::update_weights(const Eigen::MatrixXd &reservoir_state, const Eigen::MatrixXd &target) {[m
     Eigen::MatrixXd I = Eigen::MatrixXd::Identity(reservoir_state.rows(), reservoir_state.cols());[m
     Eigen::MatrixXd X_T = reservoir_state.transpose();[m
     //output matrix weights[m
[36m@@ -89,8 +88,8 @@[m [mvoid reservoir_computer::update_weights(const Eigen::MatrixXd& reservoir_state,[m
  * @param input The input data[m
  * @return A vector representing[m
  */[m
[31m-Eigen::MatrixXd reservoir_computer:: calculate_reservoir_evolution(const Eigen::MatrixXd& state_space, const Eigen::MatrixXd& input)[m
[31m-{[m
[32m+[m[32mEigen::MatrixXd[m
[32m+[m[32mreservoir_computer::calculate_reservoir_evolution(const Eigen::MatrixXd &state_space, const Eigen::MatrixXd &input) {[m
     //Wendson had it as -leakage_rate * state_space + (leakage_rate * tanh(W*state_space+W_in*input)[m
     //might also need to add a bias vector (thesis pg. 20, 21, 22) after input[m
     return (1 - leakage_rate) * state_space + leakage_rate * (W * state_space + W_in * input).unaryExpr(&hypertan);[m
