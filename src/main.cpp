
#include <vector>
#include <chrono>

#include <eigen3/Eigen/Dense>

#include "id.h"
#include "robot.h"


int main(int argc, char *argv[])
{

    
    std::string param_file = "../Robot_model/robot_parameter.json"; // change json file for different test case.
    
    Robot robot;
    robot.buildRobotFromFile(param_file);
    int DOFS= robot.getDofs(); 

    //otherwise it returns always the same number
    srand((unsigned int) time(0));

    Eigen::VectorXd q_test;
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;

    q_test.resize(DOFS);
    dq.resize(DOFS);
    ddq.resize(DOFS);

    q_test.setZero();

    q_test<<1.57,0,0,0,0,0; // add as per no. of link

    Eigen::VectorXd g;
    
    g.resize(DOFS);
    ID model_g(robot);

    dq.setZero();
    ddq.setZero();

    g = model_g.rnea(q_test, dq, ddq);

    std::cout << "--------------------Tau vector--------------------" << std::endl;
    std::cout << g << std::endl;

    return 0;

}
