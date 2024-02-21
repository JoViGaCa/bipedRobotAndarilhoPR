#include "state_space_controller/state_space_controller.h"
#include <iostream>

    // State Space Model

    void StateSpaceModel::setSystem(const Eigen::MatrixXd& newA, const Eigen::MatrixXd& newB, const Eigen::MatrixXd& newC, const Eigen::MatrixXd& newD){
        A = newA;
        B = newB;
        C = newC;
        D = newD;
        state = Eigen::VectorXd::Zero(A.rows());
    }

    void StateSpaceModel::updateState(const Eigen::VectorXd& input, double dt){
        state += (A * state + B * input) * dt;

    }

    Eigen::VectorXd StateSpaceModel::getOutput(const Eigen::VectorXd& input){
        return C * state + D * input;
    }

    // State Space Model With Observer

    void StateSpaceModelWithObserver::setSystem(const Eigen::MatrixXd& newA, const Eigen::MatrixXd& newB, const Eigen::MatrixXd& newC, const Eigen::MatrixXd& newD, const Eigen::MatrixXd& newL){
        A = newA;
        B = newB;
        C = newC;
        D = newD;
        L = newL;
        state = Eigen::VectorXd::Zero(A.rows());
    }

    void StateSpaceModelWithObserver::updateState(const Eigen::VectorXd& input, const Eigen::MatrixXd& sensorMeasurements, double dt){
        Eigen::VectorXd innovation = sensorMeasurements - (C * state);
        state += (A * state + B * input + L* innovation)*dt;

    }

    Eigen::VectorXd StateSpaceModelWithObserver::getOutput(const Eigen::VectorXd& input){
        return C * state + D * input;
    }

    // State Space Controller With Integral Action

    void StateSpaceControllerWithIntegral::setGains(const Eigen::MatrixXd& newK, const Eigen::MatrixXd& newKi){
        K = newK;
        Ki = newKi;
        integralError = Eigen::VectorXd::Zero(Ki.rows());
    }

    Eigen::VectorXd StateSpaceControllerWithIntegral::calculateControlInput(const Eigen::VectorXd outputState, const Eigen::VectorXd estimatedState, const Eigen::MatrixXd out, const Eigen::VectorXd& setpoint, double dt){
        Eigen::VectorXd error(1);

        error << setpoint - out*outputState;    

        integralError += error * dt;


        return -K * estimatedState - Ki * integralError;
    }

    // Kalman Filter 

    void KalmanFilter::predict(const Eigen::VectorXd& newInput, double dt){
        u = newInput;
        x = (A * x + B * u)*dt;  // u is the control input, if applicable
        P = (A * P * A.transpose() + Q*0.2)*dt;
    }

    void KalmanFilter::update(const Eigen::VectorXd& z){
        Eigen::MatrixXd S = C * P * C.transpose() + R;
        K = P * C.transpose() * S.inverse();  // Kalman gain
        x = x + K * (z - C * x);  // z is the measurement
        P = (Eigen::MatrixXd::Identity(A.rows(), A.cols()) - K * C) * P ;
    }

    void KalmanFilter::setComponents(const Eigen::MatrixXd& newA, const Eigen::MatrixXd& newB,
                                    const Eigen::MatrixXd& newC, const Eigen::MatrixXd& newQ,
                                    const Eigen::MatrixXd& newR){
        A = newA;
        B = newB;
        C = newC;
        Q = newQ;
        R = newR;
        P = Eigen::MatrixXd::Identity(A.rows(),A.cols())*0.2;
        x = Eigen::VectorXd::Zero(A.rows());
        u = Eigen::VectorXd::Zero(B.cols());
    }

    Eigen::VectorXd KalmanFilter::getStateEstimate(){
        return x;
    }



