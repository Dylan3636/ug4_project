#ifndef PID_H
#define PID_H
class PID{
    double p_coefficient;
    double i_coefficient;
    double d_coefficient;
    int window_size;
    int window_index;
    double integral_window[];


    double previous_p_error;
    double integral_error;
    public:
        PID(
            const double& p_coefficient,
            const double& i_coefficient,
            const double& d_coefficient,
            const int& window_size);
        
        double update_error(
            const double p_error,
            const double delta_time
        );
};
#endif