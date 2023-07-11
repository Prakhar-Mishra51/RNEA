#ifndef ID_H
#define ID_H

#include <iostream>
#include <vector>
#include <math.h>

#include <eigen3/Eigen/Dense>

#include "robot.h"

class ID{

public:
    ID(Robot robot);//constructor
    ~ID();//destructor

    Eigen::VectorXd rnea(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &ddq);//RNEA function

private:

    int NB; //no.of bodies(link)
    Eigen::VectorXd theta_;//DH param
    Eigen::VectorXd d_;//DH param
    Eigen::VectorXd a_;//DH param
    Eigen::VectorXd alpha_;//DH param
    Eigen::VectorXd tau_;                     //tau_ after projection
    std::vector<Eigen::VectorXd> v(int NB); //spatial velocity
    std::vector<Eigen::VectorXd> a(int NB);//spatial acceleration
    std::vector<Eigen::VectorXd> f(int NB); //spatial force.
    
    Robot robot_;

    void jcalc(const double &qi, Eigen::Matrix<double, 6, 6>& XJ, Eigen::VectorXd& S); // jcalc for calculation of S and XJ.

    void xtcalc(const double ai, const double di, const double alphai,  Eigen::Matrix<double, 6, 6>& XT ); //for calculation of XT

    Eigen::Matrix<double, 6, 6> crm(const Eigen::VectorXd& v); //crm  spatial cross-product operator (motion)

    Eigen::Matrix<double, 6, 6> crf(const Eigen::VectorXd& v);//crf  spatial cross-product operator (force)

    Eigen::Matrix<double, 6, 6> mci(double m, const Eigen::VectorXd& c, const Eigen::Matrix3d& I);//mcI  spatial rigid-body inertia from mass, CoM and rotational inertia.


};

#endif
