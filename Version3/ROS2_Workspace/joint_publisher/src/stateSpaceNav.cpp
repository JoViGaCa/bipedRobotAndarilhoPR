#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class stateSpaceNav: public rclcpp::Node{
    public:
        stateSpaceNav():Node("stateSpaceNav"){

            traj.joint_names.resize(10);
            traj.joint_names[0] = "quadril_dir_x_joint";
            traj.joint_names[1] = "quadril_dir_y_joint";
            traj.joint_names[2] = "quadril_esq_x_joint";
            traj.joint_names[3] = "quadril_esq_y_joint";
            traj.joint_names[4] = "joelho_dir_joint";
            traj.joint_names[5] = "joelho_esq_joint";
            traj.joint_names[6] = "torn_dir_y_joint";
            traj.joint_names[7] = "torn_esq_y_joint";
            traj.joint_names[8] = "torn_dir_x_joint";
            traj.joint_names[9] = "torn_esq_x_joint";
            traj.header.frame_id="tronco_link";
            traj.points.resize(1);
            traj.points[0].positions={0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

            this->joint_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/set_joint_trajectory", 50);
            this->sensor1 = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin1/out",50,std::bind(&stateSpaceNav::sensor1CallBack,this, _1));
            this->sensor2 = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin2/out",50,std::bind(&stateSpaceNav::sensor2CallBack,this, _1));

            timer_ = this->create_wall_timer(50ms, std::bind(&stateSpaceNav::timerCallBack, this));

        }

        void sensor1CallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            for(int i=0; i < 10;i++){
                sensor1_x_in[i] = sensor1_x_in[i+1];
                sensor1_y_in[i] = sensor1_y_in[i+1];
            }
            sensor1_x_in[10] = value->linear_acceleration.x;
            sensor1_y_in[10] = value->linear_acceleration.y;
            sensor1_x_out[10] = 0;
            sensor1_y_out[10] = 0;
            
            for(int i = 0; i < 11; i++){
                sensor1_x_out[10] += filter_a[i]*sensor1_x_in[10-i];
                sensor1_y_out[10] += filter_a[i]*sensor1_y_in[10-i];
            }

            for(int i = 1; i < 10; i++){
                sensor1_x_out[10] += filter_b[i]*sensor1_x_out[10-i];
                sensor1_y_out[10] += filter_b[i]*sensor1_y_out[10-i];
            }

            sensor1_x_out[10] = sensor1_x_out[10]*filter_b[0];
            sensor1_y_out[10] = sensor1_y_out[10]*filter_b[0]; 

            for(int i = 0; i < 10; i++){
                sensor1_x_out[i] = sensor1_x_out[i+1];
                sensor1_y_out[i] = sensor1_y_out[i+1];
            }

        }

        void sensor2CallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            for(int i=0; i < 10;i++){
                sensor2_x_in[i] = sensor2_x_in[i+1];
                sensor2_y_in[i] = sensor2_y_in[i+1];
            }
            sensor2_x_in[10] = value->linear_acceleration.x;
            sensor2_y_in[10] = value->linear_acceleration.y;
            sensor2_x_out[10] = 0;
            sensor2_y_out[10] = 0;
            
            for(int i = 0; i < 11; i++){
                sensor2_x_out[10] += filter_a[i]*sensor2_x_in[10-i];
                sensor2_y_out[10] += filter_a[i]*sensor2_y_in[10-i];
            }

            for(int i = 1; i < 10; i++){
                sensor2_x_out[10] += filter_b[i]*sensor2_x_out[10-i];
                sensor2_y_out[10] += filter_b[i]*sensor2_y_out[10-i];
            }

            sensor2_x_out[10] = sensor2_x_out[10]*filter_b[0];
            sensor2_y_out[10] = sensor2_y_out[10]*filter_b[0]; 

            for(int i = 0; i < 10; i++){
                sensor2_x_out[i] = sensor2_x_out[i+1];
                sensor2_y_out[i] = sensor2_y_out[i+1];
            }

        }

        void timerCallBack(){
            if(agachar == 1){
                center_mass[2] = center_mass[2] - dist;
            } else { 
                stimateState();
                getZMP();
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
            zmp_y = center_mass[0] - 0.16*((sensor1_y_out[10] + sensor2_y_out[10])/2)/9.8;
        }

        void stimateState(){
            // ajustar valores para delay
            accely_value[1] = accely_value[0];
            accely_value[0] = (sensor1_y_out[10] + sensor2_y_out[10])/2;

            stimated_cm[2] = stimated_cm[1];
            stimated_cm[2] = stimated_cm[0];

            stimated_vel[1] = stimated_vel[0];

            // aplica lógica
            stimated_cm[0] =  navOutCm[1]*stimated_cm[1] + navOutCm[2]*stimated_cm[2] + navInCm[0]*accely_value[0] + navInCm[0];
            stimated_vel[0] = navOutVel[1]*stimated_vel[1] + navInVel*accely_value[0];

            // Manda para o controlador
            state_vector[0] = stimated_cm[0];
            state_vector[1] = stimated_vel[0];
            state_vector[2] = accely_value[0];

        }

        void applyControl(){
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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%d - %.5f - %.5f - %.5f", count, center_mass[0], pref[count], zmp_y);
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
        double sensor1_y_in[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double sensor1_y_out[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double sensor1_x_in[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double sensor1_x_out[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double sensor2_y_in[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double sensor2_y_out[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double sensor2_x_in[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double sensor2_x_out[11] = {0,0,0,0,0,0,0,0,0,0,0};
        double filter_a[11] = {1.0000,-3.5249,6.6114,-8.0046,6.8123,-4.1900,1.8711,-0.5948,0.1282,-0.0169,0.0010};
        double filter_b[11] = {0.0001,0.0009,0.0041,0.0109,0.0191,0.0229,0.0191,0.0109,0.0041,0.0009,0.0001};

        // Cinemática Inversa
        double k_dir = -0.038, k_esq = 0.038, h = -0.01, q = 0.02, leg=-0.07;
        double pe_dir[3] = {0.038, 0, 0};
        double pe_esq[3] = {-0.038, 0, 0};
        double center_mass[3] = {0, 0, 0.19};
        double dist = 0.001, t = 0;

        // Controlador
        double state_vector[3] = {0,0,0}, state_back = 0;
        double matrixA[3][3] = {{1, 0.05, 0.0013},{0, 1, 0.05},{0, 0, 1}};
        double matrixB[3] = {0, 0.0013, 0.05};
        double matrixC[3] = {1, 0, -0.0163};
        double matrixK[3] = {0.9511, 1.9367, 1.9562};
        double zmp_y = 0;
        double aux1[3] = {0,0,0}, aux2[3] = {0,0,0}, aux3[3] = {0,0,0};

        // Navegação Inercial do centro de massa 
        double navInCm[2] = {0.005, 0.005};
        double navOutCm[3] = {1, -2, 1};
        double accely_value[2] = {0,0};
        double stimated_cm[3] = {0,0,0};

        // Navegação Invercial velocidade
        double navInVel = 0.1;
        double navOutVel[2] = {1, -1};
        double stimated_vel[2] = {0, 0};


        // Controlador
        double pref[125] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, 0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000, -0.038000};
        int count = 0, agachar = 1;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);     
    rclcpp::spin(std::make_shared<stateSpaceNav>()); 
    rclcpp::shutdown();
    return 0;
}