#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <chrono>

using namespace std::chrono_literals;

class jointPublisher: public rclcpp::Node{
    public:
        jointPublisher():Node("jointPublisher"){

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
            traj.header.frame_id="base_link";
            traj.points.resize(1);
            traj.points[0].positions={0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            this->joint_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/set_joint_trajectory", 50);
            timer_ = this->create_wall_timer(100ms, std::bind(&jointPublisher::timerCallBack, this));

        }

        void timerCallBack(){
            x_dir = -pe_dir[0]+center_mass[0];
            y_dir = -pe_dir[1]+center_mass[1];
            z_dir = -pe_dir[2]+center_mass[2];
            x_esq = -pe_esq[0]+center_mass[0];
            y_esq = -pe_esq[1]+center_mass[1];
            z_esq = -pe_esq[2]+center_mass[2];

            aux1_dir = pow((x_dir-k_dir),2) + pow((z_dir-h),2);
            aux1_esq = pow((x_esq-k_esq),2) + pow((z_esq-h),2);

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


            alpha_dir = atan((x_dir - k_dir)/(z_dir - h));
            alpha_esq = atan((x_esq - k_esq)/(z_esq - h));

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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Center of mass: %.4f || %.4f || %.4f \n", center_mass[0], center_mass[1], center_mass[2]);
            joint_pub->publish(traj);
            center_mass[2] = center_mass[2] - dist;
            if(center_mass[2] <= 3.6){
                dist = 0;
                center_mass[0] = 0.6*sin(t);
                t = t + 0.01; 
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Center of mass: %.4f || %.4f || %.4f \n", center_mass[0], center_mass[1], center_mass[2]);
            }
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        trajectory_msgs::msg::JointTrajectory traj;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub;
        double alpha_dir = 0, beta_dir = 0, gama_dir = 0, delta_dir = 0, zeta_dir = 0;
        double alpha_esq = 0, beta_esq = 0, gama_esq = 0, delta_esq = 0, zeta_esq = 0;
        double aux1_dir = 0, aux2_dir = 0, aux3_dir = 0, aux4_dir = 0;
        double aux1_esq = 0, aux2_esq = 0, aux3_esq = 0, aux4_esq = 0;
        double x_dir = 0, y_dir = 0, z_dir = 0;
        double x_esq = 0, y_esq = 0, z_esq = 0;
        // Valores obtidos na simulação Rviz
        double k_dir = -0.9, k_esq = 0.9, h = 0, q = 0.92, leg=-1.08;
        //double k_dir = -0.038, k_esq = 0.038, h = -0.01, q = 0.02, leg=-0.07;
        double pe_dir[3] = {0.9, 0, 0};
        double pe_esq[3] = {-0.9, 0, 0};
        double center_mass[3] = {0, 0, 4};
        double dist = 0.001, t = 0;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);     
    rclcpp::spin(std::make_shared<jointPublisher>()); 
    rclcpp::shutdown();
    return 0;
}