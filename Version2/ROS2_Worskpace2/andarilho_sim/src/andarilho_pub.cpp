#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <functional>
#include <memory>
#include <vector>
#include <iostream>
#include <sensor_msgs/msg/imu.hpp>
#include <gazebo_msgs/msg/link_state.hpp>
//#include <gazebo_msgs/GetLinkState.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64.hpp>



using std::placeholders::_1;
using namespace std::chrono_literals;


class AndarilhoPub : public rclcpp::Node{                      

    public:
        AndarilhoPub() : Node ("Andarilho_pub"){      
            using std::placeholders::_1;
            this->joint_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/set_joint_trajectory", 50);
            this->sensor1 = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin1/out",50,std::bind(&AndarilhoPub::sensor1CallBack,this, _1));
            this->sensor2 = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin2/out",50,std::bind(&AndarilhoPub::sensor2CallBack,this, _1));
            timer_ = this->create_wall_timer(55ms, std::bind(&AndarilhoPub::timerCallBack, this));

            traj.joint_names.resize(8);
            traj.joint_names[0] = "quadril_dir_x_joint";
            traj.joint_names[1] = "quadril_dir_y_joint";
            traj.joint_names[2] = "quadril_esq_x_joint";
            traj.joint_names[3] = "quadril_esq_y_joint";
            traj.joint_names[4] = "joelho_dir_joint";
            traj.joint_names[5] = "joelho_esq_joint";
            traj.joint_names[6] = "torn_dir_joint";
            traj.joint_names[7] = "torn_esq_joint";
            traj.header.frame_id = "tronco_link";
            traj.points.resize(1);
            traj.points[0].positions = {0, 0, 0, 0, 0, 0, 0, 0};

            for(int i = 0; i < 252; i++){
                preview_action[i] = 0;
                preview_actionx[i] = 0;
                if (i == (252-Nx)){
                    for(int k = 0; k < Nx; k++){
                        prefx[k] = prefx[k] + prefx[251];
                    }
                }
                for (int j = 0; j < Ny; j++){
                    preview_action[i] += Gd*pref[(i+j)%252];
                }
                for(int j = 0; j< Nx; j++){
                    preview_actionx[i] += Gdx*prefx[(i+j)%252];
                }
            }

            for(int i = 0; i < Nx; i++){
                prefx[i] = prefx[i] - prefx[251];
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Iniciando Controle");

        }

        void sensor1CallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            a1 = value->linear_acceleration.y;
            ax1 = value->linear_acceleration.x;
            az1 = value->linear_acceleration.z;
        }

        void sensor2CallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            a2 = value->linear_acceleration.y;
            ax2 = value->linear_acceleration.x;
            az2 = value->linear_acceleration.z;
        }

        void timerCallBack(){
                
        
            if(control == 1){
                //calcular zmp obtido

                if(k < 10){
                a[k] = (a1 + a2)/2;
                ax[k] = (ax1 + ax2)/2;
                az[k] = (az1 + az2)/2;
                } else {
                    for(int i = 0; i < 9; i++){
                        a[i] = a[i+1];
                        ax[i] = ax[i+1];
                        az[i] = az[i+1];
                    }
                    a[9] = (a1 + a2)/2;
                    ax[9] = (ax1 + ax2)/2;
                    az[9] = (az2 + az2)/2;
                }

                filtered_a = 0;
                filtered_ax = 0;
                filtered_az = 0;
                for(int i = 0; i < 10; i++){
                    filtered_a += a[i];
                    filtered_ax += ax[i];
                    filtered_az += az[i];
                }
                filtered_a = filtered_a/10;
                filtered_ax = filtered_ax/10;
                filtered_az = filtered_az/10;

                sideAngle = atan(filtered_a/filtered_az);
                frontAngle = atan(filtered_ax/filtered_az);

                filtered_a = filtered_a*cos(abs(sideAngle));
                filtered_ax = filtered_ax*cos(abs(frontAngle));      
               
                calc_zmp = state_vector[0] - (r*filtered_a)/g;
                calc_zmp_x = state_vector_x[0] - (r*filtered_ax)/g;


                //soma com vetorC*vetorU
                aux = multiVxV(vector_C , state_vector);
                auxx = multiVxV(vector_C, state_vector_x);
                aux = calc_zmp - aux;
                auxx = calc_zmp_x - auxx;

                //Multiplica por L
                result_L[0] = vector_L[0]*aux;
                result_L[1] = vector_L[1]*aux;
                result_L[2] = vector_L[2]*aux;
                result_L_x[0] = vector_L[0]*auxx;
                result_L_x[1] = vector_L[1]*auxx;
                result_L_x[2] = vector_L[2]*auxx;

                // soma result_L com state_vector
                result_L[0] = -result_L[0] + state_vector[0];
                result_L[1] = -result_L[1] + state_vector[1];
                result_L[2] = -result_L[2] + state_vector[2];
                result_L_x[0] = -result_L_x[0] + state_vector_x[0];
                result_L_x[1] = -result_L_x[1] + state_vector_x[1];
                result_L_x[2] = -result_L_x[2] + state_vector_x[2];

                // Multiplica result_L por vector_C
                aux2 = multiVxV(vector_C, result_L);
                aux2x = multiVxV(vector_C, result_L_x);

                // soma pref com aux2
                aux3 = aux2 - pref[k];
                aux3x = aux2x - prefx[k];

                // somatório
                somatorio += aux3;
                somatoriox += aux3x;

                // multiplica por Gi
                aux4 = Gi*somatorio;
                aux4x = Gix*somatoriox;

                // multiplica result_L por Gx
                aux5 = multiVxV(Gx, result_L);
                aux5x = multiVxV(Gxx, result_L_x);


                //soma de aux7
                aux6 = -preview_action[k] - aux4 - aux5;
                aux6x = -preview_actionx[k] - aux4x - aux5x;

                // multiplica aux6 por b
                result_B[0] = vector_B[0]*aux6;
                result_B[1] = vector_B[1]*aux6;
                result_B[2] = vector_B[2]*aux6;
                result_B_x[0] = vector_B[0]*aux6x;
                result_B_x[1] = vector_B[1]*aux6x;
                result_B_x[2]= vector_B[2]*aux6x;

                
                //multiplica result_B pela matrix_A
                state_vector[0] = multiVxV(matrix_A[0], result_B);
                state_vector[1] = multiVxV(matrix_A[1], result_B);
                state_vector[2] = multiVxV(matrix_A[2], result_B);
                state_vector_x[0] = multiVxV(matrix_A[0], result_B_x);
                state_vector_x[1] = multiVxV(matrix_A[1], result_B_x);
                state_vector_x[2] = multiVxV(matrix_A[2], result_B_x);

                //atualiza o valor do posCM[1]
                
                    posCM[1] = state_vector[0];
                    posCM[0] = state_vector_x[0];
            }


            // calcula a distância do passo
            distPDE = 0.01;

            if(agachar == 1){
                posCM[2] -= 0.02/252;
            }

            if (walk == 1){
                // definir posições dos pés para o movimento
                if (k > (7 + cicle*63) && k < (14 + 63*cicle)){
                    posPD[2] = altura*(-pow((count/3),2) + 1);
                    count++;
                    posPD[0] += distPDE/(6);

                } else {
                    posPD[2] = 0;
                }

                if (k > (14 + 63*cicle) && k < (40 + 63*cicle)){
                    count = -2;
                }


                if (k > (40 + 63*cicle) && k < (47 + 63*cicle)){
                    posPE[2] = altura*(-pow((count/3),2) + 1);
                    count++;
                    posPE[0] += distPDE/(6);
                } else {
                    posPE[2] = 0;
                }

                if(k > (47 + 63*cicle)){
                    count = -2;
                }

                }
            
            

            // Calcular as posições dos angulos, cinemática reversa
            ang_quad_dir_y = atan(0.05/(abs(posCM[1]-posPD[1])));
            dif_reta_dir = (posCM[0] - posPD[0]);
            retaH_dir = posCM[2] - dQ*sin(ang_quad_dir_y) - 0.03 - posPD[2];
            retaH_dir2 = pow(retaH_dir,2) + pow(dif_reta_dir,2);
            ang_joel_dir = acos((retaH_dir2/(-2*leg2)) +1);
            ang_torn_dir = 90*toRad - ang_joel_dir/2 + asin(dif_reta_dir/retaH_dir);
            ang_quad_dir_x = 90*toRad - ang_joel_dir/2 - asin(dif_reta_dir/retaH_dir);

            ang_quad_esq_y = atan(0.05/(abs(posCM[1]-posPE[1])));
            dif_reta_esq = (posCM[0] - posPE[0]);
            retaH_esq = posCM[2] - dQ*sin(ang_quad_esq_y) -0.03 -posPE[2];
            retaH_esq2 = pow(retaH_esq,2) + pow(dif_reta_esq,2);
            ang_joel_esq = acos(((retaH_esq2)/(-2*leg2)) +1);
            ang_torn_esq = 90*toRad - ang_joel_esq/2 + asin(dif_reta_esq/retaH_esq);
            ang_quad_esq_x = 90*toRad - ang_joel_esq/2 - asin(dif_reta_esq/retaH_esq);

            //ROS_INFO("Ângulo quadril eixo-y: %.4f - %.4f", ang_quad_dir_y,ang_quad_esq_y);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Centro de Massa: X:%.4f - Y:%.4f - Z:%.4f", posCM[0], posCM[1], posCM[2]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pé direito: X:%.4f - Y:%.4f - Z:%.4f", posPD[0], posPD[1], posPD[2]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pé esquerdo: X:%.4f - Y:%.4f - Z:%.4f", posPE[0], posPE[1], posPE[2]);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ângulo quadril eixo-y: %.4f - %.4f", ang_quad_dir_y*toAngle,ang_quad_esq_y*toAngle);
            //ROS_INFO("RetaH: %.4f - %.4f", retaH_dir, retaH_esq);
            //ROS_INFO("Ângulo quadril eixo-x: %.4f - %.4f", ang_quad_dir_x, ang_quad_esq_x);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ângulo quadril eixo-x: %.4f - %.4f", ang_quad_dir_x*toAngle, ang_quad_esq_x*toAngle);
            //ROS_INFO("Ângulo joelho: %.4f - %.4f", ang_joel_dir, ang_joel_esq);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ângulo joelho: %.4f - %.4f", ang_joel_dir*toAngle, ang_joel_esq*toAngle);
            //ROS_INFO("Ângulo tornozelo: %.4f - %.4f", ang_torn_dir, ang_torn_esq);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ângulo tornozelo: %.4f - %.4f", ang_torn_dir*toAngle, ang_torn_esq*toAngle);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n ------------------------------------------------------------ \n\n");
            //ROS_INFO("PosCM[1] DistPDE: %.4f - %.4f", posCM[1], sensor_dir.linear_acceleration.y);
            //ROS_INFO("PosCm - PosPD - PosPE: %.4f - %.4f - %.4f", posCM[0], posPD[0], posPE[0]);
            //ROS_INFO("DIf_reta_dir - esq:  %.4f -  %.4f", dif_reta_dir, dif_reta_esq);
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%.5f - %.5f - %.5f - %.5f - %.5f - %.5f", posCM[0], posCM[1], prefx[k], pref[k], calc_zmp_x, calc_zmp);
            
            

            traj.points[0].positions[0] = -ang_quad_dir_x;
            traj.points[0].positions[1] = (ang_quad_dir_y - offset);
            traj.points[0].positions[2] = -ang_quad_esq_x;
            traj.points[0].positions[3] = -(ang_quad_esq_y - offset);
            traj.points[0].positions[4] = 180*toRad - ang_joel_dir;
            traj.points[0].positions[5] = 180*toRad - ang_joel_esq;
            traj.points[0].positions[6] = -ang_torn_dir;
            traj.points[0].positions[7] = -ang_torn_esq;

            if(k == (63 + 63*cicle)){
                cicle = cicle + 1;
            }

            k += 1;
            if(k == 252){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Aqui");
                k = 0;
                cicle = 0;
                walk = 1;
                control = 1;
                agachar = 0;
            }


            joint_pub->publish(traj);


            joint_pub->publish(traj);


        }



        double multiVxV(double* v1, double* v2){
            if(sizeof(v1) != sizeof(v2)){
                //ROS_INFO("Erro função multiVxV: tamanhos diferentes");
                return -99;
            }
            double result;
            result = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
            return result;
        }




    private:
        const double toRad = 3.14159/180;
        const double toAngle = 180/3.14159;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub;
        trajectory_msgs::msg::JointTrajectory traj;
        std_msgs::msg::Float64 pub_cm, pub_zmp, pub_pref;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sensor1;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sensor2;
        sensor_msgs::msg::Imu sensor_dir;
        sensor_msgs::msg::Imu sensor_esq;
        std_msgs::msg::Float64 msg;
        // movimentação da perna
        const double leg = 0.07;
        double ang_quad_dir_x, ang_quad_dir_y, ang_quad_esq_x, ang_quad_esq_y;
        double ang_joel_dir, ang_joel_esq, ang_torn_dir, ang_torn_esq;
        double matrix_A [3][3] = {{1.0156, 0.0251, -0.0156}, {1.2540, 1.0156, -1.2540}, {0, 0, 1}};
        double vector_B [3] = {-0.0001, -0.0156, 0.0250};
        double vector_C [3] = {0, 0, 1};
        double result_L [3] = {0, 0, 0}, result_B [3] = {0, 0, 0};
        double result_L_x [3] = {0, 0, 0}, result_B_x [3] = {0, 0, 0};
        double vector_L [3] = { -10421, -129.9237, 107.0006};
        double state_vector [3] = {0, 0, 0};
        double state_vector_x [3] = {0, 0, 0};
        double Gi = 0.009, Gx[3] = {0, 0, 0.00075}, Gd = 15;
        double Gix = 0.0008, Gxx[3] = {0, 0, 0.0025}, Gdx = 25;
        double a1 = 0, a2 = 0, ax1 = 0, ax2 = 0, az1 = 0, az2 = 0;
        double posCM[3] = {0,0,0.22};
        double posPD[3] = {0,0.038,0};
        double posPE[3] = {0,-0.038,0};
        double offset = atan(0.05/0.038);
        double dQ = 0.0628;
        double leg2 = leg*leg;
        double retaH_dir, retaH_esq;
        double retaH_dir2, retaH_esq2;
        double dif_reta_dir, dif_reta_esq;
        double pref[252] = {0.000000, -0.006678, -0.013084, -0.018971, -0.024150, -0.028496, -0.031964, -0.034576, -0.036416, -0.037607, -0.038291, -0.038614, -0.038704, -0.038671, -0.038593, -0.038526, -0.038500, -0.038526, -0.038593, -0.038671, -0.038704, -0.038614, -0.038291, -0.037607, -0.036416, -0.034576, -0.031964, -0.028496, -0.024150, -0.018972, -0.013084, -0.006679, -0.000000, 0.006678, 0.013084, 0.018971, 0.024150, 0.028496, 0.031964, 0.034576, 0.036416, 0.037607, 0.038291, 0.038614, 0.038704, 0.038671, 0.038593, 0.038526, 0.038500, 0.038526, 0.038593, 0.038671, 0.038704, 0.038614, 0.038291, 0.037607, 0.036416, 0.034576, 0.031964, 0.028497, 0.024150, 0.018972, 0.013084, 0.006679, 0.000000, -0.006678, -0.013083, -0.018971, -0.024150, -0.028496, -0.031964, -0.034576, -0.036416, -0.037607, -0.038291, -0.038614, -0.038704, -0.038671, -0.038593, -0.038526, -0.038500, -0.038526, -0.038593, -0.038671, -0.038704, -0.038614, -0.038291, -0.037607, -0.036416, -0.034576, -0.031964, -0.028497, -0.024150, -0.018972, -0.013084, -0.006679, -0.000001, 0.006678, 0.013083, 0.018971, 0.024149, 0.028496, 0.031963, 0.034576, 0.036416, 0.037607, 0.038291, 0.038614, 0.038704, 0.038671, 0.038593, 0.038526, 0.038500, 0.038526, 0.038593, 0.038671, 0.038704, 0.038614, 0.038291, 0.037607, 0.036416, 0.034576, 0.031964, 0.028497, 0.024150, 0.018972, 0.013084, 0.006679, 0.000001, -0.006678, -0.013083, -0.018971, -0.024149, -0.028496, -0.031963, -0.034576, -0.036416, -0.037607, -0.038291, -0.038614, -0.038704, -0.038671, -0.038593, -0.038526, -0.038500, -0.038526, -0.038593, -0.038671, -0.038704, -0.038614, -0.038291, -0.037607, -0.036416, -0.034576, -0.031964, -0.028497, -0.024150, -0.018972, -0.013085, -0.006679, -0.000001, 0.006678, 0.013083, 0.018971, 0.024149, 0.028496, 0.031963, 0.034576, 0.036416, 0.037607, 0.038291, 0.038614, 0.038704, 0.038671, 0.038593, 0.038526, 0.038500, 0.038526, 0.038593, 0.038671, 0.038704, 0.038614, 0.038291, 0.037607, 0.036416, 0.034576, 0.031964, 0.028497, 0.024151, 0.018972, 0.013085, 0.006679, 0.000001, -0.006678, -0.013083, -0.018971, -0.024149, -0.028496, -0.031963, -0.034576, -0.036416, -0.037607, -0.038291, -0.038614, -0.038704, -0.038671, -0.038593, -0.038526, -0.038500, -0.038526, -0.038593, -0.038671, -0.038704, -0.038614, -0.038291, -0.037607, -0.036416, -0.034576, -0.031964, -0.028497, -0.024151, -0.018972, -0.013085, -0.006680, -0.000001, 0.006677, 0.013083, 0.018971, 0.024149, 0.028496, 0.031963, 0.034576, 0.036416, 0.037607, 0.038291, 0.038614, 0.038500, 0.038526, 0.038593, 0.038671, 0.038704, 0.038614, 0.038291, 0.037607, 0.036416, 0.034576, 0.031964, 0.028496, 0.024150, 0.018972, 0.013084, 0.006679};
        double preview_action[252];
        double prefx[252] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000800, 0.001600, 0.002400, 0.003200, 0.004000, 0.004800, 0.005600, 0.006400, 0.007200, 0.008000, 0.008800, 0.009600, 0.010400, 0.011200, 0.012000, 0.012800, 0.013600, 0.014400, 0.015200, 0.016000, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.016800, 0.017600, 0.018400, 0.019200, 0.020000, 0.020800, 0.021600, 0.022400, 0.023200, 0.024000, 0.024800, 0.025600, 0.026400, 0.027200, 0.028000, 0.028800, 0.029600, 0.030400, 0.031200, 0.032000, 0.032800, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.033600, 0.034400, 0.035200, 0.036000, 0.036800, 0.037600, 0.038400, 0.039200, 0.040000, 0.040800, 0.041600, 0.042400, 0.043200, 0.044000, 0.044800, 0.045600, 0.046400, 0.047200, 0.048000, 0.048800, 0.049600, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.050400, 0.051200, 0.052000, 0.052800, 0.053600, 0.054400, 0.055200, 0.056000, 0.056800, 0.057600, 0.058400, 0.059200, 0.060000, 0.060800, 0.061600, 0.062400, 0.063200, 0.064000, 0.064800, 0.065600, 0.066400, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200, 0.067200};
        double preview_actionx[252];
        double somatorio = 0, somatoriox = 0, aux2x, aux3x, aux4x, aux5x, aux6x, aux7x, auxx, calc_zmp_x;
        double r = 0.2, g = -9.8, aux, aux2, aux3, calc_zmp, aux4, aux5, aux6, aux7, sideAngle, frontAngle;
        double distPDE = 0, quad_offset = 5*toRad, altura = 0.0005, count = -6;
        double a [10] = {0,0,0,0,0,0,0,0,0,0};
        double ax [10] = {0,0,0,0,0,0,0,0,0,0};
        double az [10] = {0,0,0,0,0,0,0,0,0,0};
        double filtered_a, filtered_ax, filtered_az;
        int walk = 0, control = 0, agachar = 1, cicle = 0;
        int  Ny = 8, Nx = 50, k = 0;

};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);     
    rclcpp::spin(std::make_shared<AndarilhoPub>()); 
    rclcpp::shutdown();
    return 0;
}
