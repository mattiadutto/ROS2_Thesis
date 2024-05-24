#ifndef MPC_DIFFDRIVE_FBLIN_H
#define MPC_DIFFDRIVE_FBLIN_H

#include <eigen3/Eigen/Dense> 

#include "nav2_custom_controller/GUROBIsolver.h"
#include "nav2_custom_controller/fblin_unicycle.h"

#define GUROBI_LICENSEID 2520144
#define GUROBI_USERNAME  "Mario"

class MPC_diffDrive_fblin
{

public:
    MPC_diffDrive_fblin();
    ~MPC_diffDrive_fblin();

    void set_robotParams(double wheelVelMax, double wheelVelMin, double wheelRadius, double track);
    void set_MPCparams(double samplingTime, int predictionHorizon, double q, double r, int maxInfeasibleSolution);
    void set_MPCparams(double samplingTime, int predictionHorizon, double q, double r,
                  const std::vector<double>& variableLB, const std::vector<double>& variableUB, int maxInfeasibleSolution);
    void set_FBLINparams(double samplingTime, double pointPdistance);

    bool initialize();
    bool executeMPCcontroller();
    bool executeLinearizationController();

    void set_actualRobotState(const Eigen::VectorXd& actRobotState);
    void set_referenceRobotState(const Eigen::VectorXd& refRobotState);
    void set_obstacle_matrices(const Eigen::MatrixXd& matrix, const Eigen::VectorXd& vector);
    void get_actualMPCstate(Eigen::VectorXd& state);
    void get_referenceMPCstate(Eigen::VectorXd& state);
    void get_actualMPCControl(Eigen::VectorXd& control);

    void get_actualControl(double& linVelocity, double& angVelocity);
    void get_predicted_states(std::vector<double> &predicted_x,std::vector<double> &predicted_y,std::vector<double> &predicted_theta);

    void get_status(int& status);
    void get_objective_value(double& value);


private:
    // MPC parameters
    int _N;
    double _MPC_Ts;
    double _q, _r, _k, _p;
    int _maxInfeasibleSol, _infeasibleSolCnt;
    bool _MPCparamsInitialized;

    Eigen::MatrixXd _plant_A, _plant_B;
    std::vector<double> _lowerBound, _upperBound;

    Eigen::MatrixXd _Acal, _Bcal;
    Eigen::MatrixXd _Qcal, _Rcal;
    Eigen::MatrixXd _H;
    Eigen::VectorXd _f;
    Eigen::MatrixXd _Ain_vel;
    Eigen::VectorXd _Bin_vel;

    Eigen::MatrixXd _H_obst;
    Eigen::VectorXd _L_obst;
    Eigen::MatrixXd _A_obst;
    Eigen::VectorXd _B_obst;
    Eigen::MatrixXd _fblin_states;

    Eigen::MatrixXd _obst_matrix;
    Eigen::VectorXd _obst_vector;

    Eigen::VectorXd _actRobotState, _refRobotState, _predictRobotState;
    std::vector<double> _predicted_x,_predicted_y,_predicted_theta;
    Eigen::VectorXd _actMPCstate, _refMPCstate, _optimVect;

    GUROBIsolver* _solver;

    bool _constraints_received;

    // Feedback linearization parameters
    double _fblin_Ts, _Pdist;

    fblin_unicycle *_fblinController, *_fblinSimController;
    bool _FBLINparamsInitialized;

    // Robot parameters
    double _wheelVelMax, _wheelVelMin;
    double _track, _wheelRadius;
    bool _robotParamsInitialized;

    // Controller variables
    bool _controllerInitialized;
    double linearVelocity, angularVelocity;

    int _optimizerStatus;

    double objectiveValue;

    // Private member functions
    void compute_AcalMatrix();
    void compute_BcalMatrix();
    void compute_QcalMatrix();
    void compute_RcalMatrix();
    void compute_objectiveMatrix();
    void compute_wheelVelocityConstraint();
    void compute_ObstacleConstraint(const Eigen::MatrixXd& matrix, const Eigen::VectorXd& vector);


    void saveMatrixToFile(std::string fileName, Eigen::MatrixXd matrix);
};


#endif //MPC_DIFFDRIVE_FBLIN_H