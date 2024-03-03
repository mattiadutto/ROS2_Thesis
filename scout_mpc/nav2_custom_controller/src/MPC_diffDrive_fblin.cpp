#include "MPC_diffDrive_fblin.h"

#include <cmath>
#include <fstream>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

MPC_diffDrive_fblin::MPC_diffDrive_fblin()
 {
    // Initialize class variables
    _robotParamsInitialized = false;
    _MPCparamsInitialized = false;
    _FBLINparamsInitialized = false;
    _controllerInitialized = false;

    linearVelocity = angularVelocity = 0.0;

    // Initialize pointers
    _solver = NULL;
    _fblinController = _fblinSimController = NULL;
}

MPC_diffDrive_fblin::~MPC_diffDrive_fblin()
 {
    // Destroy solver object
    if (_solver)
    {
        delete _solver;
        _solver = NULL;
    }

    // Destroy linearization object
    if (_fblinController)
    {
        delete _fblinController;
        _fblinController = NULL;
    }
    if (_fblinSimController)
    {
        delete _fblinSimController;
        _fblinSimController = NULL;
    }
}

void MPC_diffDrive_fblin::set_MPCparams(double samplingTime, int predictionHorizon, double q, double r, int maxInfeasibleSolution) {
    std::vector<double> lb(2, -INFINITY);
    std::vector<double> ub(2, +INFINITY);

    // Set MPC params
    set_MPCparams(samplingTime, predictionHorizon, q, r, lb, ub, maxInfeasibleSolution);
}

void MPC_diffDrive_fblin::set_MPCparams(double samplingTime, int predictionHorizon, double q, double r,
                                        const std::vector<double>& variableLB, const std::vector<double>& variableUB, int maxInfeasibleSolution) {
    // Set MPC parameters
    _q = q;
    _r = r;
    _MPC_Ts = samplingTime;
    _N = predictionHorizon;
    _maxInfeasibleSol = maxInfeasibleSolution;
    _infeasibleSolCnt = 0;

    // Set variable 

    // .reserve() to store at least 2*_N elements without needing to reallocate memory
    // elements added with .insert() is costly and to be more efficient with .reserve(2*N) we can
    // preallocate enough memory to hold the elements that will be insterted, thus no reallocation in 
    // in the for loop when .instert() is called
    _lowerBound.reserve(2*_N);
    _upperBound.reserve(2*_N);

    for (auto i=0; i<_N; i++) 
    {
        // append and the end of _lowerBound vector the elements from variableLB vector, _N number of times
        _lowerBound.insert(_lowerBound.end(), variableLB.begin(), variableLB.end());
        _upperBound.insert(_upperBound.end(), variableUB.begin(), variableUB.end());
    }

    // Set the initialization flag
    _MPCparamsInitialized = true;
}

void MPC_diffDrive_fblin::set_FBLINparams(double samplingTime, double pointPdistance) {
    // Set feedback linearization parameters
    _fblin_Ts = samplingTime;
    _Pdist = pointPdistance;

    // Set the initialization flag
    _FBLINparamsInitialized = true;
}

void MPC_diffDrive_fblin::set_robotParams(double wheelVelMax, double wheelVelMin, double wheelRadius, double track) {
    // Set robot parameters
    _wheelVelMax = wheelVelMax;
    _wheelVelMin = wheelVelMin;
    _wheelRadius = wheelRadius;
    _track = track;

    // Set the initialization flag
    _robotParamsInitialized = true;
}

bool MPC_diffDrive_fblin::initialize() {
    /** Preliminary checks */
    if (!_robotParamsInitialized)
    {
        std::cout << "[MPC_diffDrive_fblin.initialize] Call set_robotParams() before calling initialize()" << std::endl;
        return false;
    }
    if (!_FBLINparamsInitialized)
    {
        std::cout << "[MPC_diffDrive_fblin.initialize] Call set_FBLINparams() before calling initialize()" << std::endl;
        return false;
    }
    if (!_MPCparamsInitialized)
    {
        std::cout << "[MPC_diffDrive_fblin.initialize] Call set_MPCparams() before calling initialize()" << std::endl;
        return false;
    }
    if (std::remainder(_MPC_Ts, _fblin_Ts)>=1.0e-15)
    {
        std::cout << "[MPC_diffDrive_fblin.initialize] MPC and feedback linearization sampling times should be multiple" << std::endl;
        return false;
    }

    // Initialize plant matrices according to the discrete state space model of the system after feedback linearization
    _plant_A = Eigen::MatrixXd::Identity(2,2);// [ 1 0; 0 1]
    _plant_B = _MPC_Ts*Eigen::MatrixXd::Identity(2,2); // [Ts 0; 0 Ts]

    // Initialize MPC controller parameters
    _k = -1.0/(2.0*_MPC_Ts);

    // equation is 6.12 in the thesis paper, in that equation consider A=1, B= TauMPC, Q=q,R=r and K=k 
    _p = (_q+pow(_k, 2.0)*_r)/(1.0-pow(1.0+_MPC_Ts*_k, 2.0));

    // Initialize MPC controller matrices
    compute_AcalMatrix();
    compute_BcalMatrix();
    compute_QcalMatrix();
    compute_RcalMatrix();

    _H = Eigen::MatrixXd::Zero(2*_N, 2*_N);
    _f = Eigen::VectorXd::Zero(2*_N);
    _Ain_vel = Eigen::MatrixXd::Zero(2*2*_N, 2*_N);
    _Bin_vel = Eigen::VectorXd::Zero(2*2*_N);

    // Initialize actual and reference data
    _actRobotState = Eigen::VectorXd::Zero(3); // column vector of 3 rows with 0 value ( 0 will be x, 1 will be y and 3 will be theta)
    _refRobotState = Eigen::VectorXd::Zero(3);
    _predictRobotState = Eigen::VectorXd::Zero(3*(_N+1)); // if we consider N = 3 it will have 12 entries
    _refMPCstate = Eigen::VectorXd::Zero(2);
    _actMPCstate = Eigen::VectorXd::Zero(2);
    _optimVect = Eigen::VectorXd::Zero(2*_N); // creates column vector of size twice the prediction horizon as it 
                                              // stores xp_dot and yp_dot
                                              // and it populates each row of the 2*N by 1 with 0 value

    // Initialize the solver
    _solver = new GUROBIsolver(GUROBI_LICENSEID, GUROBI_USERNAME);
    if (!_solver->initProblem(2*_N, _lowerBound, _upperBound))
    {
        std::cout << "[MPC_diffDrive_fblin.initialize] Error initializing the solver" << std::endl;
        return false;
    }

    // Initialize the linearization controller
    _fblinController = new fblin_unicycle(_Pdist);
    _fblinSimController = new fblin_unicycle(_Pdist);

    // Set the initialization flag
    _controllerInitialized = true;

    
    return true;
}

bool MPC_diffDrive_fblin::executeMPCcontroller() {
    /** Preliminary checks */
    if (!_controllerInitialized)
    {
        std::cout << "[MPC_diffDrive_fblin.executeMPCcontroller] Call initialize() before calling executeMPCcontroller()" << std::endl;
        return false;
    }

    // Compute the prediction of the plant states based on the previous step control vector
    double act_x, act_y, act_theta;
    // at first pass k=0 positions 0,1,2 of predictRobotState will be 0 
    // at k = 1 actRobotState position 0,1,2 will contain values of x y theta computed at k=0
    _predictRobotState(0) = act_x = _actRobotState(0);
    _predictRobotState(1) = act_y = _actRobotState(1);
    _predictRobotState(2) = act_theta = _actRobotState(2);

// this for loop will make the predictions for the x y theta states based on the given
// prediction horizon and store them in predictRobotState
// considering N = 3 therefore  this loop will execute 2 times to compute the states at time k+1 k+2 ahead
    for (auto k=0; k<_N; k++) {

        // increment i up to 20 if MPC_Ts/fblin_Ts = 0.2/0.01 = 20 
        // this for loop will execute 20 times with the same k value (since Ts of MPC is 20times more than fblin)
        for (auto i=0; i<_MPC_Ts/_fblin_Ts; i++) {

            // Update the linearizing controller state
            _fblinSimController->set_unicycleState(act_x, act_y, act_theta);

            // Compute the robot velocities
            double v, w; // used by the robot

            // the first (_optimVect(2*k) passes the xp_dot given by 2*k so at each iteratation it passes the next value)
            // at k = 1 it passes optimVector positions 2 and 3 (it should be xpdot and ypdot)
            // and save the velocities in v and w
            _fblinSimController->control_transformation(_optimVect(2*k), _optimVect(2*k+1), v, w);

            // Compute the next robot state for the current k

            // here are the kinematic equations in discrete time to get x,y,theta from the velocities
            
            act_x += v*_fblin_Ts*std::cos(act_theta+w*_fblin_Ts/2.0);
            act_y += v*_fblin_Ts*std::sin(act_theta+w*_fblin_Ts/2.0) ;
            act_theta += w*_fblin_Ts;
        }

        // Store the computed robot state
        _predictRobotState(3*(k+1)) = act_x; // at k = 0 it will store x y theta in 3 4 and 5 row 
                                             // at k = 1 it will be  in positions 6 7 8
                                            // at k = 2 (last one if N =3) in positions 9 10 11 (as predictRobot state has 11 positions )
        _predictRobotState(3*(k+1)+1) = act_y;
        _predictRobotState(3*(k+1)+2) = act_theta;
    }

    // Compute constraint matrices
    compute_wheelVelocityConstraint();
    std::vector<GRBConstr> wheelVelocityConstrain;
    if (!_solver->addConstraint(_Ain_vel, _Bin_vel, wheelVelocityConstrain))
    {
        std::cout << "[MPC_diffDrive_fblin.executeMPCcontroller] Error setting the wheel velocity constraint" << std::endl;
        return false;
    }

    // Compute cost function matrices
    compute_objectiveMatrix();
    if (!_solver->setObjective(_H, _f))
    {
        std::cout << "[MPC_diffDrive_fblin.executeMPCcontroller] Error setting the MPC objective" << std::endl;
        return false;
    }

    // Solve optimization problem
    int optimizerStatus;
    double objectiveValue;
    if (!_solver->solveProblem(_optimVect, objectiveValue, optimizerStatus))
    {
        _optimVect.setZero();

        std::cout << "[MPC_diffDrive_fblin.executeMPCcontroller] Error solving the optimization problem" << std::endl;
        return false;
    }
    if (optimizerStatus!=GUROBIsolver::OPTIMAL)
    {
        std::cout << "[MPC_diffDrive_fblin.executeMPCcontroller] No optimal solution found" << std::endl;

        _infeasibleSolCnt++;
        if (_infeasibleSolCnt==_maxInfeasibleSol)
        {
            _optimVect.setZero();

            std::cout << "[MPC_diffDrive_fblin.executeMPCcontroller] Maximum number of consecutive infeasible solution reached" << std::endl;
        }

        return false;
    }
    else
    {
        _infeasibleSolCnt = 0;
    }

    return true;
}

bool MPC_diffDrive_fblin::executeLinearizationController() {
    /** Preliminary checks */
    if (!_controllerInitialized) {
        std::cout << "[MPC_diffDrive_fblin.executeLinearizationController] Call initialize() before calling executeLinearizationController()" << std::endl;
        return false;
    }

    // Update the controller state
    
    // sets the current x y and theta of the robot and save them as x y theta for the feedback lin class
    _fblinController->set_unicycleState(_actRobotState(0), _actRobotState(1), _actRobotState(2));

    // Execute the feedback linearization law
    // this should obtain v and w from xp dot and yp_dot

    // in our case _optimVect is the optimal controls (xp_dot and yp_dot) computed by the 
    // MPC optimisation
    // If we were  to use only feedback linearization xp_dot is computed as (xp_ref - xp) * proportional gain

    // while the MPC provides us with the optimal control given the prediction horizon
    // so that we obtain our final v and w from the computed MPC control with control_transformation function
    _fblinController->control_transformation(_optimVect(0), _optimVect(1), linearVelocity, angularVelocity);

    return true;
}

void MPC_diffDrive_fblin::set_actualRobotState(const Eigen::VectorXd& actRobotState)
{
    if (actRobotState.size()!=3)
    {
        std::cout << "[MPC_diffDrive_fblin.set_actualRobotState] The actState variable has the wrong size" << std::endl;

        _actRobotState = actRobotState.segment(0, 3);
    }
    else
    {
        // _actRobotState contains x and y current
        _actRobotState = actRobotState;
        // actMPCstate will contain current xp and yp computed from current x and y by feedback lin
        _fblinController->ouput_transformation(_actMPCstate(0), _actMPCstate(1));

    }
}

void MPC_diffDrive_fblin::set_referenceRobotState(const Eigen::VectorXd& refRobotState)
{
    if (refRobotState.size()!=3)
    {
        std::cout << "[MPC_diffDrive_fblin.set_referenceRobotState] The refState variable has the wrong size" << std::endl;

        _refRobotState = refRobotState.segment(0, 3);
    }
    else
    {
        _refRobotState = refRobotState;
        // this will compute xp_ref and yp_ref by passing x_ref,y_ref and theta_ref points from
        // the path and and store them in refMPCstate
        // refRobotState is coming from the given path
        _fblinController->reference_transformation(_refRobotState(0), _refRobotState(1),
                                                   _refRobotState(2), _refMPCstate(0), _refMPCstate(1));
    }
}

void MPC_diffDrive_fblin::get_actualMPCControl(Eigen::VectorXd& control)
{
    control = _optimVect.head(2);
}

void MPC_diffDrive_fblin::get_actualMPCstate(Eigen::VectorXd& state)
{
    state = _actMPCstate;
}

void MPC_diffDrive_fblin::get_referenceMPCstate(Eigen::VectorXd& state)
{
    state = _actMPCstate;
}

void MPC_diffDrive_fblin::get_actualControl(double &linVelocity, double &angVelocity)
{
    linVelocity = linearVelocity;
    angVelocity = angularVelocity;
}


/** Private methods */
void MPC_diffDrive_fblin::compute_AcalMatrix() {
    // Initialize Acal matrix
    _Acal = Eigen::MatrixXd::Zero((_N+1)*2, 2);

    // Compute Acal matrix
    // select top left 2x2 block of elements from the matrix and assign to that block 2x2 Identity matrix
    // [1 0 ; 0 1]

    _Acal.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    for (int k=0; k<_N+1; k++)
    {
        // _plan_A is defined as 2x2 identity matrix
        _Acal.block(2*k, 0, 2, 2) = _plant_A.pow(k);
    }

    // Check matrix
//    saveMatrixToFile("Acal_matrix.csv", _Acal);
}

void MPC_diffDrive_fblin::compute_BcalMatrix() {
    // Initialize Bcal matrix
    _Bcal = Eigen::MatrixXd::Zero((_N+1)*2, _N*2);

    // Compute Bcal matrix
    for (int i=1; i<=_N; i++)
        for (int j=1; j<=i; j++)
            _Bcal.block(2*i, 2*(j-1), 2, 2) = _plant_A.pow(i-j)*_plant_B;

    // Check matrix
//    saveMatrixToFile("Bcal_matrix.csv", _Bcal);
}

void MPC_diffDrive_fblin::compute_QcalMatrix() {
    // Initialize Qcal matrix
    _Qcal = Eigen::MatrixXd::Zero((_N+1)*2, (_N+1)*2);

    // Compute Qcal matrix
    _Qcal.diagonal() << Eigen::VectorXd::Ones(_N*2)*_q, Eigen::VectorXd::Ones(2)*_p;

    // Check matrix
//    saveMatrixToFile("Qcal_matrix.csv", _Qcal);
}

void MPC_diffDrive_fblin::compute_RcalMatrix() {
    // Initialize Rcal matrix
    _Rcal = Eigen::MatrixXd::Zero(_N*2, _N*2);

    // Compute Rcal matrix
    _Rcal.diagonal() = Eigen::VectorXd::Ones(_N*2)*_r;

    // Check matrix
//    saveMatrixToFile("Rcal_matrix.csv", _Rcal);
}

void MPC_diffDrive_fblin::compute_objectiveMatrix() {
    // Initialize H and f matrices
    _H = Eigen::MatrixXd::Zero(2*_N, 2*_N);
    _f = Eigen::VectorXd::Zero(_H.rows());

    // Compute reference vector
    Eigen::VectorXd _refStateVect = Eigen::VectorXd::Zero(2*(_N+1));
    for (auto i=0; i<_N+1; i++) {
        _refStateVect.block(i*2, 0, 2, 1) = _refMPCstate;
    }

    // Compute H and f matrices
    _H = _Bcal.transpose()*_Qcal*_Bcal+_Rcal;
    _f = (_Acal*_actMPCstate-_refStateVect).transpose()*_Qcal*_Bcal;

    // Check matrix
//    saveMatrixToFile("H_matrix.csv", _H);
//    saveMatrixToFile("f_matrix.csv", _f);
}

void MPC_diffDrive_fblin::compute_wheelVelocityConstraint() {
    // Initialize Ain_vel and Bin_vel matrices
    _Ain_vel = Eigen::MatrixXd::Zero(2*2*_N, 2*_N);
    _Bin_vel = Eigen::VectorXd::Zero(2*2*_N);

    // Compute constraint matrices
    for (auto k=0; k<_N; k++)
     {
        // _predictRobotState(3*k+2) is the predicted theta angle at time step k
        Eigen::Matrix2d Abar {{2.0*std::cos(_predictRobotState(3*k+2))-_track/_Pdist*std::sin(_predictRobotState(3*k+2)),
                               2.0*std::sin(_predictRobotState(3*k+2))+_track/_Pdist*std::cos(_predictRobotState(3*k+2))},
                              {2.0*std::cos(_predictRobotState(3*k+2))+_track/_Pdist*std::sin(_predictRobotState(3*k+2)),
                               2.0*std::sin(_predictRobotState(3*k+2))-_track/_Pdist*std::cos(_predictRobotState(3*k+2))}};
        _Ain_vel.block(2*k, 2*k, 2, 2) = Abar;
        _Ain_vel.block(2*(k+_N), 2*k, 2, 2) = -Abar;

        _Bin_vel(2*k) = _wheelVelMax*2.0*_wheelRadius;
        _Bin_vel(2*k+1) = _wheelVelMax*2.0*_wheelRadius;
        _Bin_vel(2*(k+_N)) = -_wheelVelMin*2.0*_wheelRadius;
        _Bin_vel(2*(k+_N)+1) = -_wheelVelMin*2.0*_wheelRadius;
    }

    // Check matrix
//    saveMatrixToFile("Ain_vel_matrix.csv", _Ain_vel);
//    saveMatrixToFile("Bin_vel_matrix.csv", _Bin_vel);
}

void MPC_diffDrive_fblin::saveMatrixToFile(std::string fileName, Eigen::MatrixXd matrix) {
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file.close();
    }
}
