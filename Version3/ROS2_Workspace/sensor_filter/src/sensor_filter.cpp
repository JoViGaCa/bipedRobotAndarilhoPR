#include "sensor_filter/sensor_filter.h"

double sensor_filter::applyFilterX(double recentValue){
    for(int i=0; i < 10;i++){
        sensor_x_in[i] = sensor_x_in[i+1];
    }
    sensor_x_in[10] = recentValue;
    sensor_x_out[10] = 0;
    
    for(int i = 0; i < 11; i++){
        sensor_x_out[10] += filter_a[i]*sensor_x_in[10-i];
    }

    for(int i = 1; i < 10; i++){
        sensor_x_out[10] += filter_b[i]*sensor_x_out[10-i];
    }

    sensor_x_out[10] = sensor_x_out[10]*filter_b[0];

    for(int i = 0; i < 10; i++){
        sensor_x_out[i] = sensor_x_out[i+1];
    }
    return sensor_x_out[10];
}

double sensor_filter::applyFilterY(double recentValue){
    for(int i=0; i < 10;i++){
        sensor_y_in[i] = sensor_y_in[i+1];
    }
    sensor_y_in[10] = recentValue;
    sensor_y_out[10] = 0;
    
    for(int i = 0; i < 11; i++){
        sensor_y_out[10] += filter_a[i]*sensor_y_in[10-i];
    }

    for(int i = 1; i < 10; i++){
        sensor_y_out[10] += filter_b[i]*sensor_y_out[10-i];
    }

    sensor_y_out[10] = sensor_y_out[10]*filter_b[0];

    for(int i = 0; i < 10; i++){
        sensor_y_out[i] = sensor_y_out[i+1];
    }

    return sensor_y_out[10];
}
