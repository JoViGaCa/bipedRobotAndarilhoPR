#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <chrono>
#include "stimate_state/stimate_state.h"
#include "state_space_controller/state_space_controller.h"
#include <eigen3/Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono_literals;

class modelKasaei2019: public rclcpp::Node{
    public:
        modelKasaei2019():Node("modelKasaei2019"){

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
            traj.header.frame_id= "base_link";
            traj.points.resize(1);
            traj.points[0].positions={0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

            this->joint_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/set_joint_trajectory", 50);
            this->com_pub = this->create_publisher<std_msgs::msg::Float64>("/center_of_mass", 50);
            this->sensor_com_pub = this->create_publisher<std_msgs::msg::Float64>("/sensor_center_of_mass", 50);
            this->reference_pub = this->create_publisher<std_msgs::msg::Float64>("/reference_value", 50);
            this->accel_pub = this->create_publisher<std_msgs::msg::Float64>("/acceleration_of_mass", 50);
            this->sensor_accel_pub = this->create_publisher<std_msgs::msg::Float64>("/sensor_acceleration_of_mass", 50);
            this->sensor1 = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin1/out",50,std::bind(&modelKasaei2019::sensor1CallBack,this, _1));
            this->sensor2 = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin2/out",50,std::bind(&modelKasaei2019::sensor2CallBack,this, _1));
            
            Eigen::MatrixXd A(2,2);
            Eigen::MatrixXd B(2,1);
            Eigen::MatrixXd C(1,2);
            Eigen::MatrixXd D(1,1);
            Eigen::MatrixXd Ao(3,3);
            Eigen::MatrixXd Bo(3,1);
            Eigen::MatrixXd Co(1,3);
            Eigen::MatrixXd K(1,2);  // Proportional gain matrix
            Eigen::MatrixXd Ki(1,1); // Integral gain matrix
            Eigen::MatrixXd Ko(1,3);
            Eigen::MatrixXd Q(2,2);
            Eigen::MatrixXd R(1,1);
            Eigen::MatrixXd W(1,1);
            Eigen::MatrixXd V(1,1);
            Eigen::MatrixXd L(3,1);

            w = sqrt(3.6/9.8);

            A << -w, w, 0, w;  // Continuous Model 
            //A << 0.9702, 0.03031, 0, 1.031; // Discrete Model 
            Ao << A, Eigen::MatrixXd::Zero(2,1), -C, 0; 

            B << 0, -w; // Continuous Model
            //B << -0.0004592, -0.03077; // Discrete Model
            Bo << B, 0;
    
            C << 1, 0;
            Co << C, 0;
            
            D << 0;

            K << -116.0354, -16.2667;
            Ko << K, 0;
            Ki <<  316.2278;
            L << 0.7743, 1.2688, 0.4142;
            
            W << 0.5;
            V << 0.01;
            Q = B * V * V * B.transpose();
            R = W * W;

            kf.setComponents(A,B,C,Q,R);
            sys.setSystem(A,B,C,D);
            sysI.setGains(K,Ki);
            sysO.setSystem(Ao,Bo,Co,D,L);
            sysIO.setGains(Ko,Ki);


            timer_ = this->create_wall_timer(50ms, std::bind(&modelKasaei2019::timerCallBack, this));

        }

        void sensor1CallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            // x por conta da direção do robô
            raw_accel1 = -value->linear_acceleration.x;
            K_dir = p_dir*h_dir/(h_dir*p_dir*h_dir + r_dir);
            value_sensor1_y = value_sensor1_y + K_dir*(raw_accel1 - h_dir*value_sensor1_y);
            p_dir =(1-K_dir*h_dir)*p_dir + q_dir;

        }

        void sensor2CallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            // x por conta da direção do robô
            raw_accel2 = -value->linear_acceleration.x;
            K_esq = p_esq*h_esq/(h_esq*p_esq*h_esq + r_esq);
            value_sensor2_y = value_sensor2_y + K_esq*(raw_accel2 - h_esq*value_sensor2_y);
            p_esq =(1-K_esq*h_esq)*p_esq + q_esq;

        }

        void timerCallBack(){
            if(agachar == 1){
                if(count >= 200 && count <= 600){
                    center_mass[2] = center_mass[2] - dist;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Count = %d, CoM = %.5f", count, center_mass[2]);
                }
            } else { 
                stimateState();
                //testFilter();
                applyControlWithIntegralAction();
                //applyControlWithObserver();
                //applyControlWithKalmanFilter(); 
            }
            
            applyInverseKin();
            count++;
            if(count == 600 && agachar == 1){
                agachar = 0;
                count = 0;
            }
        }

        void testFilter(){
            accel_value.data = (value_sensor1_y + value_sensor2_y)/2;
            sensor_accel_value.data = (raw_accel1 + raw_accel2)/2;

            accel_pub->publish(accel_value);
            sensor_accel_pub->publish(sensor_accel_value);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"|%.5f|",sensor_accel_value.data);

        }

        void stimateState(){
            stimateStateObj.updateAccel(value_sensor1_y,value_sensor2_y);
            sensorCOM = stimateStateObj.getPos();
        }
        
        void applyControlWithObserver(){
            Eigen::VectorXd setpoint(1);
            setpoint << 0.5;
            Eigen::VectorXd sensorMeasurements(1);
            sensorMeasurements << sensorCOM;


            // Calculate control input with integral action
            Eigen::VectorXd controlInput = sysIO.calculateControlInput(sysO.state,sysO.state,sysO.C, setpoint, dt);

            // Update system state
            sysO.updateState(controlInput, sensorMeasurements, dt);

            // Get system output
            Eigen::VectorXd output = sysO.getOutput(controlInput);

            center_mass[0] = output(0);
            sensor_com_value.data = sensorCOM;
            reference_value.data = setpoint(0);
            com_value.data = center_mass[0];
            accel_value.data = (value_sensor1_y + value_sensor2_y)/2;
            com_pub->publish(com_value);
            sensor_com_pub->publish(sensor_com_value);
            reference_pub->publish(reference_value);
            accel_pub->publish(accel_value);


            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%d - %.5f - %.5f", count, center_mass[0], setpoint(0));

        }

        void applyControlWithIntegralAction(){
            if(count%100 == 0){
                target = -target;
            }

            Eigen::VectorXd setpoint(1);
            setpoint << target;

            // Calculate control input with integral action
            Eigen::VectorXd controlInput = sysI.calculateControlInput(sys.state,sys.state,sys.C, setpoint, dt);

            // Update system state
            sys.updateState(controlInput, dt);

            // Get system output
            Eigen::VectorXd output = sys.getOutput(controlInput);

            center_mass[0] = output(0);
            sensor_com_value.data = sensorCOM;
            reference_value.data = setpoint(0);
            accel_value.data = (value_sensor1_y + value_sensor2_y)/2;
            sensor_accel_value.data = (raw_accel1 + raw_accel2)/2;

            //com_pub->publish(com_value);
            //sensor_com_pub->publish(sensor_com_value);
            //reference_pub->publish(reference_value);
            accel_pub->publish(accel_value);
            sensor_accel_pub->publish(sensor_accel_value);


            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"|%.5f|",sensor_accel_value.data);

        }

        void applyControlWithKalmanFilter(){
            Eigen::VectorXd setpoint(1);
            setpoint << 0.5;
            Eigen::VectorXd sensorMeasurements(1);
            sensorMeasurements << sensorCOM;

            // set the input of the controller
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Setting the input");
        

            // predict and update kalman filter
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Predicting and Updating");            

            // apply to the system
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Getting the output");
            Eigen::VectorXd output(1);
            output << 0;

            
            center_mass[0] = output(0);
            sensor_com_value.data = sensorCOM;
            reference_value.data = setpoint(0);
            com_value.data = center_mass[0];
            com_pub->publish(com_value);
            sensor_com_pub->publish(sensor_com_value);
            reference_pub->publish(reference_value);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%d - %.5f - %.5f", count, center_mass[0], setpoint(0));
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
            traj.points[0].positions[1] = alpha_esq;
            traj.points[0].positions[2] = -beta_dir;
            traj.points[0].positions[3] = -beta_esq;
            traj.points[0].positions[4] = -gama_dir;
            traj.points[0].positions[5] = -gama_esq;
            traj.points[0].positions[6] = -delta_dir;
            traj.points[0].positions[7] = -delta_esq;
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
        std_msgs::msg::Float64 com_value;
        std_msgs::msg::Float64 sensor_com_value;
        std_msgs::msg::Float64 reference_value;
        std_msgs::msg::Float64 accel_value;
        std_msgs::msg::Float64 sensor_accel_value;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sensor_com_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr reference_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accel_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sensor_accel_pub;
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
        double value_sensor1_y = 0,value_sensor2_y = 0;
        double raw_accel1 = 0, raw_accel2 = 0;
        double K_dir = 0, q_dir = 10, r_dir = 40, p_dir = 0, h_dir = 1;
        double K_esq = 0, q_esq = 10, r_esq = 40, p_esq = 0, h_esq = 1;

        // Cinemática Inversa
        double k_dir = -0.9, k_esq = 0.9, h = 0, q = 0.92, leg=-1.08;
        double pe_dir[3] = {0.9, 0, 0};
        double pe_esq[3] = {-0.9, 0, 0};
        double center_mass[3] = {0, 0, 4};
        double dist = 0.001, t = 0;

        // Controlador
        int count = 0, agachar = 1;
        double w = 0, target = 0.5;
        double dt = 0.05;
        double sensorCOM = 0;
        KalmanFilter kf;
        StateSpaceModel sys;
        StateSpaceControllerWithIntegral sysI, sysIO;
        StateSpaceModelWithObserver sysO;


        // Navegação Inercial
        stimate_state stimateStateObj;


        
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);     
    rclcpp::spin(std::make_shared<modelKasaei2019>()); 
    rclcpp::shutdown();
    return 0;
}