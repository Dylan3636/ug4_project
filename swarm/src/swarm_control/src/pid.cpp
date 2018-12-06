#include <algorithm>
#include "pid.h"

PID::PID(
    const double& p_coefficient,
    const double& i_coefficient,
    const double& d_coefficient,
    const int& window_size){
        this->p_coefficient = p_coefficient;
        this->i_coefficient = i_coefficient;
        this->d_coefficient = d_coefficient;
        this->window_size = window_size;
        this->integral_window[window_size] = {};
        this->window_index = 0;

        this->previous_p_error=0;
        this->integral_error=0;
    }

    double PID::update_error(
            const double p_error,
            const double delta_time
    ){
        double d_error = (p_error-this->previous_p_error)/delta_time;
        this->integral_window[this->window_index] = d_error*delta_time;
        double i_error = std::accumulate(this->integral_window, this->integral_window+this->window_size, 0);
        this->previous_p_error = p_error;
        return p_coefficient*p_error + i_coefficient*i_error + d_coefficient*d_error;
    }