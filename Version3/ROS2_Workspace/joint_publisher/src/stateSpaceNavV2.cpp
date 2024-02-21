#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <chrono>
#include "sensor_filter/sensor_filter.h"
#include "stimate_state/stimate_state.h"
#include "state_space_controller/state_space_controller.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class stateSpaceNavV2: public rclcpp::Node{
    public:
        stateSpaceNavV2():Node("stateSpaceNavV2"){

            traj.joint_names.resize(10);
            traj.joint_names[0] = "junta_quadril_direito";
            traj.joint_names[1] = "junta_quadril_esquerdo";
            traj.joint_names[2] = "junta_perna1_direita";
            traj.joint_names[3] = "junta_perna1_esquerda";
            traj.joint_names[4] = "junta_perna2_direita";
            traj.joint_names[5] = "junta_perna2_esquerda";
            traj.joint_names[6] = "junta_torn_direito";
            traj.joint_names[7] = "junta_torn_esquerdo";
            traj.joint_names[8] = "junta_pe_direito";
            traj.joint_names[9] = "junta_pe_esquerdo";
            traj.header.frame_id="tronco_link";
            traj.points.resize(1);
            traj.points[0].positions={0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

            this->joint_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/set_joint_trajectory", 50);
            this->sensor1 = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin1/out",50,std::bind(&stateSpaceNavV2::sensor1CallBack,this, _1));
            this->sensor2 = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin2/out",50,std::bind(&stateSpaceNavV2::sensor2CallBack,this, _1));

            timer_ = this->create_wall_timer(50ms, std::bind(&stateSpaceNavV2::timerCallBack, this));

        }

        void sensor1CallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            value_sensor1_y = filtro1.applyFilterX(value->linear_acceleration.y);

        }

        void sensor2CallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            value_sensor2_y = filtro2.applyFilterY(value->linear_acceleration.y);

        }

        void timerCallBack(){
            if(agachar == 1){
                center_mass[2] = center_mass[2] - dist;
            } else { 
                getZMP();
                stimateState();
                applyControl(); 
            }
            
            applyInverseKin();
            count++;
            if(count == 30 && agachar == 1){
                agachar = 0;
                count = 0;
            }
            if(count == 125){
                count = 0;
            }
        }

        void getZMP(){
            zmp_y = center_mass[0] - 0.16*((value_sensor1_y + value_sensor2_y)/2)/9.8;
        }

        void stimateState(){
            //stimateStateObj.updateAccel(value_sensor1_y, value_sensor2_y);

            
            //state_vector[0] = stimateStateObj.stimateCenterOfMass();
            //state_vector[1] = stimateStateObj.stimateCenterVel();
            //state_vector[2] = stimateStateObj.getAccel();

        }

        void applyControl(){
            
            //center_mass[0] = stateSpaceController.applyController(state_vector[0],state_vector[1],state_vector[2],count);
            //-kx(t)
            state_back = (state_vector[0]*matrixK[0] + state_vector[1]*matrixK[1] + state_vector[2]*matrixK[2]);
            //state_back = (state_vector[0]*matrixK[0] + state_vector[1]*matrixK[1] + state_vector[2]*matrixK[2]);
            

            // B*u, and, u = (pref - state_back)
            aux1[0] = matrixB[0]*(pref[count] - state_back);
            aux1[1] = matrixB[1]*(pref[count] - state_back);
            aux1[2] = matrixB[2]*(pref[count] - state_back);
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Aux1: %.5f - %.5f - %.5f", aux1[0], aux1[1], aux1[2]);

            // A*x

            aux3[0] = matrixA[0][0]*state_vector[0] + matrixA[0][1]*state_vector[1] + matrixA[0][2]*state_vector[2];
            aux3[1] = matrixA[1][0]*state_vector[0] + matrixA[1][1]*state_vector[1] + matrixA[1][2]*state_vector[2];
            aux3[2] = matrixA[2][0]*state_vector[0] + matrixA[2][1]*state_vector[1] + matrixA[2][2]*state_vector[2];
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Aux3: %.5f - %.5f - %.5f", aux3[0], aux3[1], aux3[2]);

            state_vector[0] = aux1[0] + aux3[0];
            state_vector[1] = aux1[1] + aux3[1];
            state_vector[2] = aux1[2] + aux3[2];

            center_mass[0] = state_vector[0];

            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%d - %.5f - %.5f - %.5f", count, center_mass[0], stateSpaceController.getZMP(count), zmp_y);
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sensor Values: %.5f - %.5f", sensor1_y_out[10], sensor2_y_out[10]);

        }

        void applyInverseKin(){
            x_dir = -pe_dir[0]+center_mass[0];
            y_dir = -pe_dir[1]+center_mass[1];
            z_dir = -pe_dir[2]+center_mass[2];
            x_esq = -pe_esq[0]+center_mass[0];
            y_esq = -pe_esq[1]+center_mass[1];
            z_esq = -pe_esq[2]+center_mass[2];

            aux1_dir = pow((x_dir-k_dir),2) + pow((z_dir+h),2);
            aux1_esq = pow((x_esq-k_esq),2) + pow((z_esq+h),2);

            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Aux1 right - left: %.4f || %.4f \n", aux1_dir, aux1_esq);

            aux2_dir = (sqrt(aux1_dir) - 2*q)/(-leg);
            aux2_esq = (sqrt(aux1_esq) - 2*q)/(-leg);

            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Aux2 right - left: %.4f || %.4f \n", aux2_dir, aux2_esq);

            aux3_dir = (y_dir)/leg;
            aux3_esq = (y_esq)/leg;

            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Aux3 right - left: %.4f || %.4f \n", aux3_dir, aux3_esq);
            
            // cos gama
            aux4_dir = (pow(aux2_dir,2) + pow(aux3_dir,2) - 2)/2;
            aux4_esq = (pow(aux2_esq,2) + pow(aux3_esq,2) - 2)/2;

            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Aux4 (cos(gama)) right - left: %.4f || %.4f \n", aux4_dir, aux4_esq);

            gama_dir = abs(atan(sqrt(1-pow(aux4_dir,2))/aux4_dir));
            gama_esq = abs(atan(sqrt(1-pow(aux4_esq,2))/aux4_esq));

            beta_dir = -abs(atan(aux3_dir/aux2_dir) - atan(sin(gama_dir)/(1+cos(gama_dir))));
            beta_esq = -abs(atan(aux3_esq/aux2_esq) - atan(sin(gama_esq)/(1+cos(gama_esq))));

            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Atan right - left: %.4f || %.4f \n", atan(aux2_dir/aux3_dir), atan(aux2_esq/aux3_esq));
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Atan2 right - left: %.4f || %.4f \n", atan(sin(gama_dir)/(1+cos(gama_dir))), atan(sin(gama_esq)/(1+cos(gama_esq))));


            alpha_dir = atan((x_dir - k_dir)/(z_dir + h));
            alpha_esq = atan((x_esq - k_esq)/(z_esq + h));

            zeta_dir = - alpha_dir;
            zeta_esq = - alpha_esq;

            delta_dir = - beta_dir - gama_dir;
            delta_esq = - beta_esq - gama_esq;


            traj.points[0].positions[0] = alpha_dir;
            traj.points[0].positions[1] = beta_dir;
            traj.points[0].positions[2] = alpha_esq;
            traj.points[0].positions[3] = beta_esq;
            traj.points[0].positions[4] = gama_dir;
            traj.points[0].positions[5] = gama_esq;
            traj.points[0].positions[6] = delta_dir;
            traj.points[0].positions[7] = delta_esq;
            traj.points[0].positions[8] = zeta_dir;
            traj.points[0].positions[9] = zeta_esq;
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Right anlges: %.4f || %.4f || %.4f || %.4f || %.4f \n", alpha_dir, beta_dir, gama_dir, delta_dir, zeta_dir);
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Left anlges: %.4f || %.4f || %.4f || %.4f || %.4f \n", alpha_esq, beta_esq, gama_esq, delta_esq, zeta_esq);
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Center of mass: %.4f || %.4f || %.4f \n", center_mass[0], center_mass[1], center_mass[2]);
            joint_pub->publish(traj);
        }

    private:

        // ROS2
        rclcpp::TimerBase::SharedPtr timer_;
        trajectory_msgs::msg::JointTrajectory traj;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sensor1;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sensor2;
        sensor_msgs::msg::Imu sensor_dir;
        sensor_msgs::msg::Imu sensor_esq;

        // Cinempatica Inversa
        double alpha_dir = 0, beta_dir = 0, gama_dir = 0, delta_dir = 0, zeta_dir = 0;
        double alpha_esq = 0, beta_esq = 0, gama_esq = 0, delta_esq = 0, zeta_esq = 0;
        double aux1_dir = 0, aux2_dir = 0, aux3_dir = 0, aux4_dir = 0;
        double aux1_esq = 0, aux2_esq = 0, aux3_esq = 0, aux4_esq = 0;
        double x_dir = 0, y_dir = 0, z_dir = 0;
        double x_esq = 0, y_esq = 0, z_esq = 0;

        // Filtro
        sensor_filter filtro1;
        sensor_filter filtro2;
        double value_sensor1_y = 0,value_sensor2_y = 0;

        // Cinemática Inversa
        double k_dir = -0.038, k_esq = 0.038, h = -0.01, q = 0.02, leg=-0.07;
        double pe_dir[3] = {0.038, 0, 0};
        double pe_esq[3] = {-0.038, 0, 0};
        double center_mass[3] = {0, 0, 0.19};
        double dist = 0.001, t = 0;

        // Controlador
        //state_space_controller stateSpaceController;
        double zmp_y;
        int count = 0, agachar = 1;
        //double state_vector[3] = {0,0,0};

        double state_vector[3] = {0,0,0}, state_back = 0;
        double matrixA[3][3] = {{1, 0.05, 0.0013},{0, 1, 0.05},{0, 0, 1}};
        double matrixB[3] = {0, 0.0013, 0.05};
        double matrixC[3] = {1, 0, -0.0163};
        double matrixK[3] = {0.9511, 1.9367, 1.9562};
        double aux1[3] = {0,0,0}, aux3[3] = {0,0,0};
        double pref[125] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000};


        // Navegação Inercial
        stimate_state stimateStateObj;


        
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);     
    rclcpp::spin(std::make_shared<stateSpaceNavV2>()); 
    rclcpp::shutdown();
    return 0;
}