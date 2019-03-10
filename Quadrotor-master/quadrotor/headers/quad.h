//
//  quad.h
//  SUNNY
//
//  Created by Sunny Singh on 2/6/19.
//  Copyright © 2019 RESL. All rights reserved.
//

#ifndef quad_h
#define quad_h
#include "iostream"
#include "eigen3/Eigen/Dense"
#include"RESL/Step.h"
#include "RESL/Reset.h"
using namespace Eigen;
using namespace std;
namespace quad_dynamics{
    class Quadynamics{
        
    public :
        Quadynamics();
        bool step(RESL::Step::Request &req,
             RESL::Step::Response &res);
        bool reset(RESL::Reset::Request &req,
             RESL::Reset::Response &res);
    private:    
        double m;// mass in gms
        double g; // gravity in m/sec2
        double kd; // drag constant 0 for now
        double Ixx;
        double Iyy;
        double Izz;
        double k;
        double L;//length of arm
        double b; //to be changed
        double dt;
        // Defining Matrix
         Matrix<double, 3, 1> x;
        Matrix<double, 3, 1> xdot;
        Matrix<double, 3, 1> theta;
        Matrix<double, 3, 1> thetadot;
        Matrix<double, 3, 3> I;
        Matrix<double, 4, 1> inputs;
        Matrix<double,3,1> omega;
        MatrixXd a;
        MatrixXd omegadot;
        MatrixXd getOmega(MatrixXd theta, MatrixXd thetadot);
        MatrixXd getThetadot(Matrix<double,3,1> omega, MatrixXd theta);
        MatrixXd getThrust(MatrixXd inputs, double k);
        MatrixXd getRotation(MatrixXd theta);
        MatrixXd getAcc(MatrixXd inputs,MatrixXd theta, MatrixXd xdot, double m, double g, double k, double kd);
        MatrixXd getTorque(MatrixXd inputs, double L, double b, double k);
        MatrixXd getAngularAcc(MatrixXd inputs, Matrix<double,3,1> omega, MatrixXd I, double L, double b, double k);};
}
#endif /* quad_h */
