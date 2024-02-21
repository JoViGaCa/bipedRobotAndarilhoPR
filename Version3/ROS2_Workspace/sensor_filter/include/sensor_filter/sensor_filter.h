#include <iostream>

class sensor_filter{
    private:
        double sensor_y_in[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double sensor_y_out[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double sensor_x_in[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double sensor_x_out[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double filter_a[11] = {1.0000,-3.5249,6.6114,-8.0046,6.8123,-4.1900,1.8711,-0.5948,0.1282,-0.0169,0.0010};
        double filter_b[11] = {0.0001,0.0009,0.0041,0.0109,0.0191,0.0229,0.0191,0.0109,0.0041,0.0009,0.0001};


    public:
        double applyFilterX(double recentValue);
        double applyFilterY(double recentValue);
};