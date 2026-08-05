void magneto_sample(double x, double y, double z, double* ata, double* norm_sum, double* sample_count);
// returns nonzero on allocation failure, BAinv is left untouched
int magneto_current_calibration(float BAinv[4][3], double* ata, double norm_sum, double sample_count);