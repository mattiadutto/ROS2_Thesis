#include "GUROBIsolver.h"


GUROBIsolver::GUROBIsolver(int licenseID, std::string username) {
    // Initialize class variables
    _pEnv = NULL;
    _pModel = NULL;
    _licenseID = licenseID;
    _username = username;

    _GUROBIinitialized = false;
}

GUROBIsolver::~GUROBIsolver() {
    // Destroy model and environment
    if (!_pModel) {
        delete _pModel;
    }

    if (!_pEnv) {
        delete _pEnv;
    }
}

bool GUROBIsolver::initProblem(int numVariable, const std::vector<double>& lowerBound, const std::vector<double>& upperBound) {
    /** Preliminary checks */
    if (numVariable<=0)
    {
        std::cout << "[GUROBIsolver.initProblem] The number of variables should be greater than zero" << std::endl;

        return false;
    }
if (lowerBound.size() != numVariable || upperBound.size() != numVariable)
    {
        std::cout << "[GUROBIsolver.initProblem] The size of lowerBound upperBound vectors should be equal to the number of variables" << std::endl;

        return false;
    }

    /** Create and initialize a GUROBI environment */
    try
    {
        // Create GUROBI environment
        _pEnv = new GRBEnv(true);
        _pEnv->set(GRB_IntParam_LogToConsole, 0);
        _pEnv->set(GRB_IntParam_LicenseID, _licenseID);
        _pEnv->set(GRB_StringParam_UserName, _username);
        _pEnv->start();

        // Create GUROBI model
        _pModel = new GRBModel(_pEnv);
        _pModel->set(GRB_IntAttr_ModelSense, 1);

        // Create optimization variables without lower/upper bounds
        for (auto var=0; var<numVariable; var++) {
            if (std::isinf(lowerBound.at(var)))
            {
                _variableVect.push_back(_pModel->addVar(-GRB_INFINITY, upperBound.at(var), 0.0, GRB_CONTINUOUS));
            }
            else if (std::isinf(upperBound.at(var)))
            {
                _variableVect.push_back(_pModel->addVar(lowerBound.at(var), GRB_INFINITY, 0.0, GRB_CONTINUOUS));
            }
            else if (std::isinf(lowerBound.at(var)) && std::isfinite(upperBound.at(var)))
            {
                _variableVect.push_back(_pModel->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS));
            }
            else
            {
                _variableVect.push_back(_pModel->addVar(lowerBound.at(var), upperBound.at(var), 0.0, GRB_CONTINUOUS));
            }
        }
    } catch(GRBException e) {
        std::cout << "[GUROBIsolver.initProblem] Error initializing GUROBI environment:" << std::endl;
        std::cout << "\t [" << e.getErrorCode() << ":" << e.getMessage() << "]" << std::endl;

        _GUROBIinitialized = false;
        return false;
    }

    // GUROBI is now initialized
    _GUROBIinitialized = true;

    return true;
}

bool GUROBIsolver::setObjective(const Eigen::MatrixXd& hessian, const Eigen::VectorXd& gradient) {
// Set the objective for a minimization problem: min 0.5*xT*H*x+fT*x

    /** Preliminary checks */
    if (!_GUROBIinitialized)
    {
        std::cout << "[GUROBIsolver.setObjective] Call initProblem() before setObjective()" << std::endl;

        return false;
    }
    if ((hessian.rows()<1) || (hessian.cols()<1))
    {
        std::cout << "[GUROBIsolver.setObjective] Cannot set the objective with an empty hessian" << std::endl;

        return false;
    }
    if (gradient.size()<1)
    {
        std::cout << "[GUROBIsolver.setObjective] Cannot set the objective with an empty gradient" << std::endl;

        return false;
    }
    if (hessian.rows()!=hessian.cols())
    {
        std::cout << "[GUROBIsolver.setObjective] Cannot set the objective with a non square hessian" << std::endl;

        return false;
    }
    if (gradient.size()!=hessian.rows())
    {
        std::cout << "[GUROBIsolver.setObjective] Cannot set the objective with a gradient having wrong size" << std::endl;

        return false;
    }
    if (hessian.rows()!=static_cast<Eigen::Index>(_variableVect.size()))
    {
        std::cout << "[GUROBIsolver.setObjective] Cannot set the objective with an hessian having wrong dimensions" << std::endl;

        return false;
    }

    /** Set the objective */
    try
    {
        // Add quadratic elements of the objective
        GRBQuadExpr obj = 0;
        for (auto row=0; row<static_cast<Eigen::Index>(_variableVect.size()); row++) {
            // Add quadratic elements of the objective line by line
            std::vector<GRBVar> tmpVariables(_variableVect.size(), _variableVect.at(row));

            std::vector<double> obj_row(hessian.cols());
            Eigen::Map<Eigen::RowVectorXd>(&obj_row[0], 1, hessian.cols()) = hessian.row(row)*0.5;

            obj.addTerms(&obj_row.at(0), &_variableVect.at(0), &tmpVariables.at(0), _variableVect.size());
        }

        // Add the linear part of the objective
        std::vector<double> obj_f(gradient.size());
        Eigen::VectorXd::Map(&obj_f.at(0), gradient.size()) = gradient;

        obj.addTerms(&obj_f.at(0), &_variableVect.at(0), _variableVect.size());

        _pModel->setObjective(obj, GRB_MINIMIZE);
    } catch(GRBException e) {
        std::cout << "[GUROBIsolver.setObjective] GUROBI failed setting up the objective:" << std::endl;
        std::cout << "\t [" << e.getErrorCode() << ":" << e.getMessage() << "]" << std::endl;

        return false;
    }

    return true;
}

bool GUROBIsolver::addConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, std::vector<GRBConstr>& constraint) {
    return addConstraint(A, b, "", constraint);
}

bool GUROBIsolver::addConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, std::string name, std::vector<GRBConstr>& constraint) {
// Add a linear inequality constraint: Ax <= b

    /** Preliminary checks */
    if (!_GUROBIinitialized)
    {
        std::cout << "[GUROBIsolver.addConstraint] Call initProblem() before addConstraint()" << std::endl;

        constraint.clear();
        return false;
    }
    if ((A.rows()<1) || (A.cols()<1) || (b.rows()<1) || (b.cols()<1))
    {
        std::cout << "[GUROBIsolver.addConstraint] Cannot add an empty set of inequality constraints" << std::endl;

        constraint.clear();
        return false;
    }
    if ((A.cols()!=static_cast<Eigen::Index>(_variableVect.size())) || (A.rows()!=b.rows()) || (b.cols()!=1))
    {
        std::cout << "[GUROBIsolver.addConstraint] Cannot add a set of inequality constraints having wrong dimensions" << std::endl;

        constraint.clear();
        return false;
    }

    /** Add the inequality constraint */
    try
    {
        // Add the inequality constraint
        for (auto row=0; row<A.rows(); row++) {
            // Add constraint line by line
            std::vector<double> A_row(A.cols()); Eigen::Map<Eigen::RowVectorXd>(&A_row.at(0), 1, A.cols()) = A.row(row);

            GRBLinExpr constr = 0;
            constr.addTerms(&A_row.at(0), &_variableVect.at(0), _variableVect.size());
            constraint.push_back(_pModel->addConstr(constr <= b(row), name+"["+std::to_string(row)+"]"));
        }
    } catch(GRBException e) {
        std::cout << "[GUROBIsolver.addConstraint] GUROBI failed adding a constraint:" << std::endl;
        std::cout << "\t [" << e.getErrorCode() << ":" << e.getMessage() << "]" << std::endl;

        constraint.clear();
        return false;
    }

    return true;
}

bool GUROBIsolver::modifyConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, std::vector<GRBConstr>& constraint)
{
    /** Preliminary checks */
    if (!_GUROBIinitialized)
    {
        std::cout << "[GUROBIsolver.modifyConstraint] Call initProblem() before modifyConstraint()" << std::endl;

        return false;
    }
    if (constraint.size()==0)
    {
        std::cout << "[GUROBIsolver.modifyConstraint] The set of constraints to be modified is empty" << std::endl;

        return false;
    }
    if ((A.rows()<1) || (A.cols()<1) || (b.rows()<1) || (b.cols()<1))
    {
        std::cout << "[GUROBIsolver.modifyConstraint] The new set is an empty set of inequality constraints" << std::endl;

        return false;
    }
    if ((A.cols()!=static_cast<Eigen::Index>(_variableVect.size()) || (A.rows()!=b.rows()) || (b.cols()!=1) || (A.rows()!=static_cast<Eigen::Index>(constraint.size()))))
    {
        std::cout << "[GUROBIsolver.modifyConstraint] The new set of inequality constraints have wrong dimensions" << std::endl;

        return false;
    }

    /** Modify the constraint */
    try
    {
        // Modify constraint line by line
        for (size_t constr=0; constr<constraint.size(); constr++) {
            // Modify the constraint left hand side
            for (size_t var=0; var<_variableVect.size(); var++) {
                _pModel->chgCoeff(constraint.at(constr), _variableVect.at(var), A(constr,var));
            }

            // Modify the constraint right hand side
            constraint.at(constr).set(GRB_DoubleAttr_RHS, b(constr));
        }
    } catch(GRBException e) {
        std::cout << "[GUROBIsolver.removeConstraint] GUROBI failed modifying a constraint:" << std::endl;
        std::cout << "\t [" << e.getErrorCode() << ":" << e.getMessage() << "]" << std::endl;

        return false;
    }

    return true;
}

bool GUROBIsolver::removeConstraint(std::vector<GRBConstr>& constraint) {
    /** Preliminary checks */
    if (!_GUROBIinitialized)
    {
        std::cout << "[GUROBIsolver.removeConstraint] Call initProblem() before removeConstraint()" << std::endl;

        return false;
    }
    if (constraint.size()==0)
    {
        std::cout << "[GUROBIsolver.removeConstraint] The set of constraints to be removed is empty" << std::endl;

        return false;
    }

    /** Remove the constraint */
    try
    {
        for (size_t constr=0; constr<constraint.size(); constr++) {
            _pModel->remove(constraint.at(constr));
        }
    } catch(GRBException e) {
        std::cout << "[GUROBIsolver.removeConstraint] GUROBI failed removing a constraint:" << std::endl;
        std::cout << "\t [" << e.getErrorCode() << ":" << e.getMessage() << "]" << std::endl;

        return false;
    }

    // Remove constraint objects
    constraint.clear();

    return true;
}

bool GUROBIsolver::solveProblem(Eigen::VectorXd& result, double& objectiveValue, int& optimizerStatus) {
    /** Preliminary checks */
    if (!_GUROBIinitialized)
    {
        std::cout << "[GUROBIsolver.solveProblem] Call initProblem() before solveProblem()" << std::endl;

        return false;
    }

    /** Solve the QP problem */
    try
    {
        // Solve the problem
        _pModel->optimize();

        // Check the status of the solver
        if (_pModel->get(GRB_IntAttr_Status)==GRB_OPTIMAL)
        {
            for (size_t var=0; var<_variableVect.size(); var++) {
                result(var) = _variableVect.at(var).get(GRB_DoubleAttr_X);
            }

            objectiveValue = _pModel->get(GRB_DoubleAttr_ObjVal);
            optimizerStatus = GUROBIsolver::OPTIMAL;
        }
        else if (_pModel->get(GRB_IntAttr_Status)==GRB_INFEASIBLE)
        {
            for (size_t var=0; var<_variableVect.size(); var++) {
                result(var) = 0.0;
            }

            objectiveValue = -1.0;
            optimizerStatus = GUROBIsolver::INFEASIBLE;
            return false;
        }
        else
        {
            for (size_t var=0; var<_variableVect.size(); var++) {
                result(var) = 0.0;
            }

            objectiveValue = -1.0;
            optimizerStatus = GUROBIsolver::OTHER;
            return false;
        }
    } catch(GRBException e) {
        std::cout << "[GUROBIsolver.solveProblem] GUROBI failed solving QP problem:" << std::endl;
        std::cout << "\t [" << e.getErrorCode() << ":" << e.getMessage() << "]" << std::endl;

        return false;
    }

    return true;
}
bool GUROBIsolver::writeProblem(const std::string& filename) {
    /** Preliminary checks */
    if (!_GUROBIinitialized)
    {
        std::cout << "[GUROBIsolver.writeProblem] Call initProblem() before writeProblem()" << std::endl;

        return false;
    }

    try
    {
        _pModel->write(filename+".mps");
    } catch(GRBException e) {
        std::cout << "[GUROBIsolver.writeProblem] GUROBI failed writing model to file:" << std::endl;
        std::cout << "\t [" << e.getErrorCode() << ":" << e.getMessage() << "]" << std::endl;

        return false;
    }

    return true;
}

bool GUROBIsolver::printConstraint(const std::vector<GRBConstr>& constraint) {
    /** Preliminary checks */
    if (!_GUROBIinitialized)
    {
        std::cout << "[GUROBIsolver.printConstraint] Call initProblem() before printConstraint()" << std::endl;

        return false;
    }

    try
    {
        for (size_t constr=0; constr<constraint.size(); constr++) {
            std::cout << "Constraint: " << constraint.at(constr).get(GRB_StringAttr_ConstrName) << std::endl;

            std::cout << "\t LHS: [";
            for (size_t var=0; var<_variableVect.size()-1; var++) {
                std::cout << _pModel->getCoeff(constraint.at(constr), _variableVect.at(var)) << ", ";
            }
            std::cout << _pModel->getCoeff(constraint.at(constr), _variableVect.at(_variableVect.size()-1)) << "]" << std::endl;

            std::cout << "\t RHS: " << constraint.at(constr).get(GRB_DoubleAttr_RHS) << std::endl;

            std::cout << "\t Sense: " << constraint.at(constr).get(GRB_CharAttr_Sense) << std::endl;

        }
    } catch(GRBException e) {
        std::cout << "[GUROBIsolver.writeprintConstraintProblem] GUROBI failed extracting data from a constraint:" << std::endl;
        std::cout << "\t [" << e.getErrorCode() << ":" << e.getMessage() << "]" << std::endl;

        return false;
    }

    return true;
}