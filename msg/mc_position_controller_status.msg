# Multicopter position controller status

uint64 timestamp	# time since system start (microseconds)

float32 hover_thr_estimate	# hover thrust estimate
float32 hover_thr_est_gain	# hover thrust estimator gain
float32 thr_int			# thrust integral
float32 thr_sp
float32 pos
float32 hover_thr_estimate_acc
float32 vel_dot
uint32 hover_thr_est_samples
bool hover_thr_est_init
