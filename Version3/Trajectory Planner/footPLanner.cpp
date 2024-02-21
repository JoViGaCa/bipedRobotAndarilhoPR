#include <iostream>
#include <vector>


using namespace std;

vector<double> getFootPos(double sL, double dist, double sD, double sSD, double dSD, double ts, double tm, bool firstStep){
    vector<double> trajectory;
    double pos = 0;

    //dividir o tempo maximo pela duração do passo
    int rep = (int) (tm/sD);
    //iniciar loop para percorrer sD N vezes em passos de ts
    for(int i = 0; i < rep; i++){
        for(double t = 0; t < sD; t+=ts){
        //verificar se é o pé  dar o passo
            if(firstStep){
                //se for, verificar se está em single support phase
                if(t <= sSD){
                    // se estiver, pos = -1
                    pos = -1;
                } else {
                    // se não, pos = pos+stepLengh+dist
                    pos+=sL+ dist;
                }
            } else {
                //se não for, manter a posição atual
                pos = pos;
            }
            trajectory.push_back(pos);
        }
        firstStep = !firstStep;
    }
}




int main(){

    double stepLength = 60;
    double stepWidth = 40;
    double stepDutarion = 2;
    double singleSupDu = 0.5;
    double doubleSupDu = 1.5;
    double ts = 1/20;
    double t_max = 10;
    double dist = 10;


    // calcular a posição de cada pé em função do tempo
    vector<double> dirFootPos = getFootPos(stepLength, dist, stepDutarion, singleSupDu, doubleSupDu, ts, t_max, true);
    vector<double> esqFootPos = getFootPos(stepLength, dist, stepDutarion, singleSupDu, doubleSupDu, ts, t_max, false);





    return 0;
}