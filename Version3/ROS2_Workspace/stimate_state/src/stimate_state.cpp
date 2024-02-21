#include "stimate_state/stimate_state.h"

    double stimate_state::getPos(){
        return pos[0];
    }

    double stimate_state::getVel(){
        return vel;
    }


    void stimate_state::updateAccel(double accel1, double accel2){
        accel[2] = accel[1];
        accel[1] = accel[0];
        accel[0] = (accel1 + accel2)/2;

        vel += (accel[0] + accel[1])*dt*0.5;
        pos_alt += vel*dt;

        //pos_alt += (vel)/dt + accel[0]*dt*dt/2;
        //vel += (accel[0])*dt;
        pos[0] = pos_alt;

    }
