//
//  mellinger.h
//  Mellinger
//
//  Created by Sunny Singh on 2/25/19.
//  Copyright © 2019 RESL. All rights reserved.
//

#ifndef mellinger_h
#define mellinger_h
#include <iostream>
#include "eigen3/Eigen/Dense"
using namespace Eigen;
using namespace std;
namespace mellinger_controller{
    class MellingerController{
        
        public :
        void init_controller(RESL::Mellinger::Request  &req,
         RESL::Mellinger::Response &res);
        void controller(double x, double y, double z, double xdot, double ydot, double zdot, double roll, double pitch, double yaw, double rolldot, double pitchdot, double yawdot, double acc_x, double acc_y, double acc_z);
        
        MellingerController();
    private:
        Matrix <double, 3,1> r, r_t, theta, theta_t, thetadot, thetadot_t, ep, ev, rdot, r_tdot, acc, acc_t, jerk, jerk_t, Kp, Kv, g, t, zb, t_t, zb_t, Pf_b, zb_des, e3,xc_des,yb_des,xb_des, xc,xb,yb,er,ew,hw,hw_t,Fdes,wbw,wbw_t,xc_t,yb_t,xb_t,ut ;
        Matrix<double,3,3> Rdes,R,err,Kw,Kr;
        Matrix<double,4,4> cont;
         Matrix<double,4,1>u;
          Matrix<double,4,1>rotorspeed;
        double m,time,u1_t,u1,u1_des,u2_des,u3_des,u4_des,p,q,r1,r1_t,p_t,q_t,kf,w1,w2,w3,w4,km,L;
        int check;
        
    };
}

#endif /* mellinger_h */
