#ifndef GUROBISOLVER_H
#define GUROBISOLVER_H

#include "gurobi_c++.h"
#include <eigen3/Eigen/Dense>  


class GUROBIsolver {

public:
    GUROBIsolver(int licenseID, std::string username);
    ~GUROBIsolver();

    bool initProblem(int numVariable, const std::vector<double>& lowerBound, const std::vector<double>& upperBound);

    bool setObjective(const Eigen::MatrixXd& hessian, const Eigen::VectorXd& gradient);

    bool addConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, std::vector<GRBConstr>& constraint);
    bool addConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, std::string name, std::vector<GRBConstr>& constraint);
    bool modifyConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, std::vector<GRBConstr>& constraint);
    bool removeConstraint(std::vector<GRBConstr>& constraint);

    bool solveProblem(Eigen::VectorXd& result, double& objectiveValue, int& optimizerStatus);

    bool writeProblem(const std::string& filename);
    bool printConstraint(const std::vector<GRBConstr>& constraint);
 
   enum optimizationStatus {OPTIMAL, INFEASIBLE, OTHER};
private:
    bool _GUROBIinitialized;

    GRBEnv* _pEnv;
    GRBModel* _pModel;
    int _licenseID;
    std::string _username;

    std::vector<GRBVar> _variableVect;
};


#endif //GUROBISOLVER_H
