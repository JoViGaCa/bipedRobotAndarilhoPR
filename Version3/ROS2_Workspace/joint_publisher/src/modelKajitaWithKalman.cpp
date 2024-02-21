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
#include <eigen3/Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono_literals;

class modelKajitaWithKalman: public rclcpp::Node{
    public:
        modelKajitaWithKalman():Node("modelKajitaWithKalman"){

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
            this->com_pub = this->create_publisher<std_msgs::msg::Float64>("/center_of_mass", 50);
            this->sensor_com_pub = this->create_publisher<std_msgs::msg::Float64>("/sensor_center_of_mass", 50);
            this->vel_pub = this->create_publisher<std_msgs::msg::Float64>("/velocity_of_mass", 50);
            this->sensor_vel_pub = this->create_publisher<std_msgs::msg::Float64>("/sensor_velocity_of_mass", 50);
            this->accel_pub = this->create_publisher<std_msgs::msg::Float64>("/acceleration_of_mass", 50);
            this->sensor_accel_pub = this->create_publisher<std_msgs::msg::Float64>("/sensor_acceleration_of_mass", 50);
            this->reference_pub = this->create_publisher<std_msgs::msg::Float64>("/reference_value", 50);
            this->sensor1 = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin1/out",50,std::bind(&modelKajitaWithKalman::sensor1CallBack,this, _1));
            this->sensor2 = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin2/out",50,std::bind(&modelKajitaWithKalman::sensor2CallBack,this, _1));
            
            Eigen::MatrixXd Ad(3,3);
            Eigen::MatrixXd Bd(3,1);
            Eigen::MatrixXd Ao(4,4);
            Eigen::MatrixXd Bo(4,1);
            Eigen::MatrixXd Co(1,4);
            Eigen::MatrixXd A(3,3);
            Eigen::MatrixXd B(3,1);
            Eigen::MatrixXd C(1,3);
            Eigen::MatrixXd D(1,1);
            Eigen::MatrixXd K(1,3);  // Proportional gain matrix
            Eigen::MatrixXd Ko(1,4);  // Proportional gain matrix
            Eigen::MatrixXd Ki(1,1); // Integral gain matrix
            Eigen::MatrixXd Q(3,3);
            Eigen::MatrixXd R(1,1);
            Eigen::MatrixXd W(1,1);
            Eigen::MatrixXd V(1,1);
            Eigen::MatrixXd L(1,3);
            Eigen::MatrixXd H(4,1);

            w = sqrt(3.6/9.8);
            A << 0, 1, 0, 0, 0, 1, 0, 0, 0;
            B << 0, 0, 1;
            C << 1, 0, 0;
            Ao << A, Eigen::MatrixXd::Zero(3,1), -C, 0;
            Bo << B, 0;
            Co << C, 0;
            Ad << 1 , dt, pow(dt,2)/2, 0, 1, dt, 0, 0, 1;
            Bd << pow(dt,6)/3, pow(dt,2)/2, 1;
            D << 0;
            K << 1888.0, 374.0, 032.0;
            Ko << K, 0;
            Ki << -3465.0;
            W << 40;
            V << 1;
            L << 0, 0, 1;
            Q = Bd * V * V * Bd.transpose();
            R = W * W;
            //H << 0.8777, 0.3852, -11.1285, 0.4142; // W = 1
            H << 0.0112, 0.0001, -0.1239, 0.0050; // W = 10
            //H << 0, 0, 0, 0;
            kf.setComponents(A,B,L,Q,R);
            sys.setSystem(A,B,C,D);
            sysI.setGains(K,Ki);
            sysIO.setGains(Ko, Ki);
            sysO.setSystem(Ao,Bo,Co,D,H);


            timer_ = this->create_wall_timer(50ms, std::bind(&modelKajitaWithKalman::timerCallBack, this));

        }

        void sensor1CallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            // x por conta da direção do robô
            //value_sensor1_y = -filtro1.applyFilterX(value->linear_acceleration.x);
            value_sensor1_y = -value->linear_acceleration.x;

        }

        void sensor2CallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            // x por conta da direção do robô
            //value_sensor2_y = -filtro2.applyFilterY(value->linear_acceleration.x);
            value_sensor2_y = -value->linear_acceleration.x;

        }

        void timerCallBack(){
            if(agachar == 1){
                if(count >= 200 && count <= 600){
                    center_mass[2] = center_mass[2] - dist;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Count = %d, CoM = %.5f", count, center_mass[2]);
                }
            } else { 
                stimateState();
                if(count <= 500000000){
                    applyControlWithIntegralAction();
                } else {
                //applyControlWithObserver();
                    applyControlWithKalmanFilter(); 
                }
            }
            
            applyInverseKin();
            count++;
            if(count == 600 && agachar == 1){
                agachar = 0;
                count = 0;
            }
        }

        void stimateState(){
            stimateStateObj.updateAccel(value_sensor1_y,value_sensor2_y);
            sensorVel = stimateStateObj.getVel();
            sensorCOM = stimateStateObj.getPos();
        }
        
        void applyControlWithObserver(){
            Eigen::VectorXd setpoint(1);
            setpoint << 0.5;
            Eigen::VectorXd sensorMeasurements(1);
            sensorMeasurements << sensorCOM;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Calculating Input");
            // Calculate control input with integral action
            Eigen::VectorXd controlInput = sysIO.calculateControlInput(sysO.state, sysO.state,sysO.C, setpoint, dt);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Input: %.5f", controlInput(0));

            // Update system state
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Updating System");
            sysO.updateState(controlInput, sensorMeasurements, dt);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"State: %.3f - %.3f - %.3f - %.3f", sysO.state(0), sysO.state(1), sysO.state(2), sysO.state(3));

            // Get system output
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Getting Output");
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
            Eigen::VectorXd setpoint(1);
            setpoint << 0.5;
            Eigen::VectorXd sensorMeasurements(1);
            sensorMeasurements << (value_sensor1_y + value_sensor2_y)/2;

            // Calculate control input with integral action
            Eigen::VectorXd controlInput = sysI.calculateControlInput(sys.state,sys.state,sys.C, setpoint, dt);

            // Update system state
            sys.updateState(controlInput, dt);

            // Predict and Update Kalman Filter
            kf.predict(controlInput, dt);
            kf.update(sensorMeasurements);

            // Get system output
            Eigen::VectorXd output = sys.getOutput(controlInput);
            Eigen::VectorXd kfState = kf.getStateEstimate();

            center_mass[0] = output(0);
            com_value.data = center_mass[0];
            sensor_com_value.data = kfState(0);
            vel_value.data = sys.state(1);
            sensor_vel_value.data = kfState(1);
            accel_value.data = sys.state(2);
            sensor_accel_value.data = kfState(2);
            reference_value.data = setpoint(0);
            //accel_value.data = (value_sensor1_y + value_sensor2_y)/2;


            com_pub->publish(com_value);
            sensor_com_pub->publish(sensor_com_value);

            vel_pub->publish(vel_value);
            sensor_vel_pub->publish(sensor_vel_value);

            accel_pub->publish(accel_value);
            sensor_accel_pub->publish(sensor_accel_value);

            reference_pub->publish(reference_value);
            //accel_pub->publish(accel_value);


            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%d - %.5f - %.5f", count, center_mass[0], setpoint(0));

        }

        void applyControlWithKalmanFilter(){
            Eigen::VectorXd setpoint(1);
            setpoint << 0.5;
            Eigen::VectorXd sensorMeasurements(1);
            sensorMeasurements << (value_sensor1_y + value_sensor2_y)/2;

            // set the input of the controller
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Setting the input");
            Eigen::VectorXd controllerInput = sysI.calculateControlInput(sys.state,kf.getStateEstimate(), sys.C, setpoint, dt);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Predicting and Updating");            
            kf.predict(controllerInput, dt);
            kf.update(sensorMeasurements);
            sys.updateState(controllerInput,dt);

            // apply to the system
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Getting the output");
            Eigen::VectorXd kfState;
            kfState = kf.getStateEstimate();
            Eigen::VectorXd non_observable_state;
            non_observable_state = sys.getOutput(controllerInput);

            
            center_mass[0] = non_observable_state(0);

            com_value.data = center_mass[0];
            sensor_com_value.data = kfState(0);
            vel_value.data = sys.state(1);
            sensor_vel_value.data = kfState(1);
            accel_value.data = sys.state(2);
            sensor_accel_value.data = kfState(2);
            reference_value.data = setpoint(0);
            //accel_value.data = (value_sensor1_y + value_sensor2_y)/2;


            com_pub->publish(com_value);
            sensor_com_pub->publish(sensor_com_value);

            vel_pub->publish(vel_value);
            sensor_vel_pub->publish(sensor_vel_value);

            accel_pub->publish(accel_value);
            sensor_accel_pub->publish(sensor_accel_value);

            reference_pub->publish(reference_value);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%d - %.5f - %.5f", count, kfState(0), non_observable_state(0));
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
        std_msgs::msg::Float64 vel_value;
        std_msgs::msg::Float64 sensor_vel_value;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sensor_com_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr reference_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accel_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sensor_accel_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sensor_vel_pub;
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
        double k_dir = -0.9, k_esq = 0.9, h = 0, q = 0.92, leg=-1.08;
        double pe_dir[3] = {0.9, 0, 0};
        double pe_esq[3] = {-0.9, 0, 0};
        double center_mass[3] = {0, 0, 4};
        double dist = 0.001, t = 0;

        // Controlador
        int count = 0, agachar = 1;
        double w = 0;
        double dt = 0.05;
        double sensorCOM = 0, sensorVel = 0;
        KalmanFilter kf;
        StateSpaceModel sys;
        StateSpaceControllerWithIntegral sysI, sysIO;
        StateSpaceModelWithObserver sysO;


        // Navegação Inercial
        stimate_state stimateStateObj;


        
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);     
    rclcpp::spin(std::make_shared<modelKajitaWithKalman>()); 
    rclcpp::shutdown();
    return 0;
}