import matplotlib.pyplot as mp
import math as mt

def main():
    stepLength = 0.060
    stepWidth = 0.040
    stepDuration = 2
    singleSupDu = 0.5
    doubleSupDu = 1.5
    ts = 1/20
    t_max = 10
    dist = stepLength+0.010

    pos_dir = 0
    pos_esq = 0
    passo_dir = True
    first_Step = True

    trajDir = []
    trajEsq = []
    times =[]

    # Cria array com os instantes de tempo
    for i in range(0,int(stepDuration/ts)):
        times.append(i*ts)        

    # acha quantos passos serão dados
    rep = t_max/stepDuration
    # a cada passo
    for i in range (0, int(rep)):
        # percorre os instantes de tempo daquele passo
        for t in times:
            # Se for passo com o pé direito
            if(passo_dir == 1):
                #aumenta a posição em fase de suporte único e mantém o msm caso contrário
                if(t < singleSupDu):
                    if(first_Step == 1):
                        pos_dir += (dist/2)/(singleSupDu/ts)
                    else:
                        pos_dir += (dist)/(singleSupDu/ts)
                
            else:
                #aumenta a posição em fase de suporte único e mantém o msm caso contrário
                if(t < singleSupDu):
                    if(first_Step == 1):
                        pos_esq += (dist/2)/(singleSupDu/ts)
                    else:
                        pos_esq += (dist)/(singleSupDu/ts)
                
            trajDir.append(pos_dir)
            trajEsq.append(pos_esq)

        if(first_Step == 1):
            first_Step = False
        #alterna o passo
        passo_dir = -passo_dir

    # plota as trajetórias
    real_time = []
    for i in range(0, int(t_max/ts)):
        real_time.append(i*ts)


    fig, axs = mp.subplots(2,2)
    fig.suptitle('Trajectory Generation')
    axs[0, 0].plot(real_time,trajDir,'r')
    axs[0, 0].plot(real_time,trajEsq,'k')
    axs[0, 0].set(xlabel ="Time (s)", ylabel ="Position (mm)")
    #mp.show()


    #Calcula o ponto de zero momento
    # para cada passo
    zmp = 0
    first_Step = True
    zmpTrajX = []
    passo_dir = True
    for i in range(0, int(rep)):
        #percorre os instantes de tempo
        for t in times:
            #se o tempo estiver entre 0 e Tss
            if(t < singleSupDu):
                # ZMP = current_POS_foot
                if(passo_dir == 1):
                    zmp = trajEsq[int(i*(stepDuration/ts) + t/ts)]
                else:
                    zmp = trajDir[int(i*(stepDuration/ts) + t/ts)]
            # else
            else:
                # ZMP = current_POS_foot + SL(t-Tss)/Tds
                if(passo_dir == 1):
                    if(first_Step == 1):
                        zmp = trajEsq[int(i*(stepDuration/ts) + t/ts)] + (dist/2)*(t - singleSupDu)/doubleSupDu
                    else:
                        zmp = trajEsq[int(i*(stepDuration/ts) + t/ts)] + (dist)*(t - singleSupDu)/doubleSupDu
                else:
                    if(first_Step == 1):
                        zmp = trajDir[int(i*(stepDuration/ts) + t/ts)] + (dist/2)*(t - singleSupDu)/doubleSupDu
                    else:
                        zmp = trajDir[int(i*(stepDuration/ts) + t/ts)] + (dist)*(t - singleSupDu)/doubleSupDu

            
            zmpTrajX.append(zmp)
        passo_dir = -passo_dir

    axs[1, 0].plot(real_time,zmpTrajX,'r')
    axs[1, 0].set(xlabel="Time", ylabel = "Position")
    

    # ZMP Eixo Y
    pos_dir = 0.027
    pos_esq = -0.027
    zmp = 0
    zmpTrajY = []
    passo_dir = True
    for i in range(0, int(rep)):
        #percorre os instantes de tempo
        for t in times:
            #se o tempo estiver entre 0 e Tss
            if(t < singleSupDu):
                # ZMP = current_POS_foot
                if(passo_dir == 1):
                    zmp = pos_esq
                else:
                    zmp = pos_dir
            # else
            else:
                # ZMP = current_POS_foot + SL(t-Tss)/Tds
                if(passo_dir == 1):
                    zmp = pos_esq + (pos_dir-pos_esq)*(t - singleSupDu)/doubleSupDu
                else:
                    zmp = pos_dir - (pos_dir-pos_esq)*(t - singleSupDu)/doubleSupDu
                    
            
            zmpTrajY.append(zmp)
        passo_dir = -passo_dir

    axs[0, 1].plot(real_time,zmpTrajY,'r')
    axs[0, 1].set(xlabel="Time", ylabel = "Position")
    #mp.show()


    # geral posição centro de massa
    x0 = 0
    xf = 0
    t0 = 0
    tf = 0
    posCOM = []
    pos_com = 0
    w = mt.sqrt(9.8/0.20)

    for i in range(0,int(rep)):
        #atualiza os valores de posição e tempo
        t0 = i*stepDuration
        tf = (1+i)*stepDuration - ts
        x0 = zmpTrajX[int(t0/ts)]
        xf = zmpTrajX[int(tf/ts)]
        print(str(t0) + ":" + str(tf) + ":" + str(x0) + ":" + str(xf))

        # aplica a equação a cada instante de tempo
        for t in times:
            time_now = (t+stepDuration*i)
            zmp_now = zmpTrajX[int(round(time_now/ts))]
            pos_com = zmp_now + ((zmp_now - xf)*mt.sinh(w*(time_now-t0)) + (x0-zmp_now)*mt.sinh(w*(time_now-tf)))/(mt.sinh(w*(t0-tf)))
            posCOM.append(pos_com)
        print("-----")
    
    axs[1, 1].plot(real_time,posCOM,'r')
    axs[1, 1].set(xlabel="Time", ylabel = "Position")
    mp.show()







if __name__ == "__main__":
    main()
