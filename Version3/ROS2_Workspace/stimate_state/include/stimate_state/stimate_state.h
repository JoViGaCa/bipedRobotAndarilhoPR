#include <iostream>

class stimate_state{
    private:
        // Navegação Inercial do centro de massa 
        double accel[3] = {0,0,0};
        double vel = 0, pos_alt = 0;
        double pos[3] = {0,0,0};
        double dt = 0.05;


    public:
        void updateAccel(double accel1, double accel2);
        double getPos();
        double getVel();

};