#include <iostream>
#include <eigen3/Eigen/Dense>

class StateSpaceModel {
    public:
        // State variables
        Eigen::VectorXd state;

        // System matrices
        Eigen::MatrixXd A, B, C, D;

        // Constructor
        void setSystem(const Eigen::MatrixXd& newA, const Eigen::MatrixXd& newB, const Eigen::MatrixXd& newC, const Eigen::MatrixXd& newD);

        // Update state function
        void updateState(const Eigen::VectorXd& input, double dt);

        // Output function
        Eigen::VectorXd getOutput(const Eigen::VectorXd& input);
};

class StateSpaceModelWithObserver{
    public:
        // State variables
        Eigen::VectorXd state;

        // System matrices
        Eigen::MatrixXd A, B, C, D, L;

        // Constructor
        void setSystem(const Eigen::MatrixXd& newA, const Eigen::MatrixXd& newB, const Eigen::MatrixXd& newC, const Eigen::MatrixXd& newD, const Eigen::MatrixXd& newL);

        // Update state function
        void updateState(const Eigen::VectorXd& input, const Eigen::MatrixXd& sensorMeasurements, double dt);

        // Output function
        Eigen::VectorXd getOutput(const Eigen::VectorXd& input);

};

class StateSpaceControllerWithIntegral {
    public:
        Eigen::MatrixXd K;  // Proportional gain matrix
        Eigen::MatrixXd Ki; // Integral gain matrix
        Eigen::VectorXd integralError; // Integral of error

        void setGains(const Eigen::MatrixXd& newK, const Eigen::MatrixXd& newKi);

        // Control input calculation with integral action
        Eigen::VectorXd calculateControlInput(const Eigen::VectorXd outputState, const Eigen::VectorXd estimatedState, const Eigen::MatrixXd out, const Eigen::VectorXd& setpoint, double dt);
};


class KalmanFilter {
    public:
        Eigen::MatrixXd A;  // State transition matrix
        Eigen::MatrixXd B;  // Control input matrix
        Eigen::MatrixXd C;  // Observation matrix
        Eigen::MatrixXd Q;  // Process noise covariance matrix
        Eigen::MatrixXd R;
        Eigen::VectorXd x;  // Initial state estimate
        Eigen::MatrixXd P;  // Initial state covariance matrix
        Eigen::MatrixXd K;  // Kalman gain
        Eigen::VectorXd u;  // Controller input for prediction

        void predict(const Eigen::VectorXd& newInput, double dt);
        void update(const Eigen::VectorXd& z);
        void setComponents(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                            const Eigen::MatrixXd& C, const Eigen::MatrixXd& Q,
                            const Eigen::MatrixXd& R);
        Eigen::VectorXd getStateEstimate();


};


