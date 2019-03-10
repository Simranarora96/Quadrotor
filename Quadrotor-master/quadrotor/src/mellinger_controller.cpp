#include "ros/ros.h"
#include "RESL/Mellinger.h"
#include "RESL/Step.h"

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Vector3.h"
#include "yaml-cpp/yaml.h"
#include "RESL/Reset.h"
#include "headers/mellinger.h"
#include <iostream>
#include <limits>
#include <vector>
#include <math.h>
using namespace Eigen;
using namespace std;
using namespace mellinger_controller;

MellingerController::MellingerController(){        
    
//init//
        
    };
    void MellingerController::init_controller(RESL::Mellinger::Request  &req,RESL::Mellinger::Response &res)
    {
		YAML::Node config = YAML::LoadFile("src/RESL/config/config.yaml");
        m = config["m"].as<double>();
        time = config["time"].as<double>();
        r_t<<config["r_t_x"].as<double>(), config["r_t_y"].as<double>(),config["r_t_z"].as<double>();
        theta_t<<config["theta_t_x"].as<double>(), config["theta_t_y"].as<double>(),config["theta_t_z"].as<double>();
        Kp<<config["Kp_x"].as<double>(), config["Kp_y"].as<double>(),config["Kp_z"].as<double>();
        Kv<<config["Kv_x"].as<double>(), config["Kv_y"].as<double>(),config["Kv_z"].as<double>();
        g<<0,0,config["g"].as<double>();
        Kw<<config["Kw"].as<double>(),0,0,
        0,config["Kw"].as<double>(),0,
        0,0,config["Kw"].as<double>();
        Kr<<config["Kr"].as<double>(),0,0,
        0,config["Kr"].as<double>(),0,
        0,0,config["Kr"].as<double>();
        kf=config["kf"].as<double>();
        km=config["km"].as<double>();
        L = config["L"].as<double>();
        cont<<kf,kf,kf,kf,
		        0,kf*L,0,-kf*L,
		        -kf*L,0,kf*L,0,
		        km,-km,km,-km;
		check = 0; 
		 ros::NodeHandle n;
		  ros::ServiceClient client = n.serviceClient<RESL::Reset>("reset_service");
		  RESL::Reset srv;
		  
		  if (client.call(srv))
		  {
		  	controller(srv.response.position.x,srv.response.position.y,srv.response.position.z,srv.response.linear_velocity.x,srv.response.linear_velocity.y,srv.response.linear_velocity.z, srv.response.angular_position.x,srv.response.angular_position.y,srv.response.angular_position.z,srv.response.angular_velocity.x,srv.response.angular_velocity.y,srv.response.angular_velocity.z,0,0,0);
		  }
		  else
		  {
		    ROS_ERROR("Failed to call Reset Service, cannot initiate the mellinger controller");
		  
		  }


    };

    	void MellingerController::controller(double x, double y, double z, double xdot, double ydot, double zdot, double roll, double pitch, double yaw, double rolldot, double pitchdot, double yawdot, double acc_x, double acc_y, double acc_z)
         {
    	//             ----    Calling the reset service to get the initial state of the quadcopter   ---
    	  

		  	/*     ---  fetching the initial state from the response and calculating dynamics   ---- */ 
		       r<<x,y,z;
		       rdot<<xdot,ydot,zdot;
		       theta<<roll,pitch,yaw;
		       thetadot<<rolldot,pitchdot,yawdot;
		       acc<<acc_x,acc_y,acc_z;
		       jerk = acc/time;
		       t<<acc(0,0),acc(1,0),acc(2,0)+g(2,0);
		       r_tdot = (r_t-r)/time;
		       acc_t = (r_tdot-rdot)/time;
		       jerk_t = (acc_t-acc)/time;
		       thetadot_t = (theta_t-theta)/time;
		       t_t<<acc_t(0,0),acc_t(1,0),acc_t(2,0)+g(2,0);
		       
		

		  
		        /*        ----          Calculating differentially flat outputs for r and rT          ----                   */
		        
		        /* for r */
		        /**********************************************************************************************/
		        double yaw1 = thetadot(2,0);
		        Matrix<double,3,1> phidot;
		        phidot<<0,0,yaw1; // creating phiZw
		        u1 = m*t.norm();
		        zb = t/(t.norm());
		        xc << cos(theta(2,0)), sin(theta(2,0)), 0;
		        yb = (zb.cross(xc))/(zb.cross(xc)).norm();
		        xb = yb.cross(zb);
		        R <<xb,yb,zb; // calculating Rotation in body frame w.r.t to world frame
		        hw = (m/u1)*(jerk-(zb.dot(jerk))*zb);
		        p = (-1)*hw.dot(yb);
		        q = hw.dot(xb);
		        r1 = phidot.dot(zb);
		        /**********************************************************************************************/
		        
		        /* for rT */
		        /**********************************************************************************************/
		        double phi_t  = thetadot_t(2,0);
		        Matrix<double,3,1> phidot_t;
		        phidot_t<<0,0,phi_t;
		        zb_t = t_t/(t_t.norm());
		        xc_t << cos(theta_t(2,0)), sin(theta_t(2,0)),0;
		        yb_t = (zb_t.cross(xc_t))/(zb_t.cross(xc_t)).norm();
		        xb_t = yb_t.cross(zb_t);
		        u1_t = m*t_t.norm();
		        hw_t = (m/u1_t)*(jerk_t-(zb_t.dot(jerk_t))*zb_t);
		        p_t = (-1)*hw_t.dot(yb_t);
		        q_t = hw_t.dot(xb_t);
		        r1_t = phidot_t.dot(zb_t);
		        /**********************************************************************************************/
		        
		        
		        /*        -----      calculating ep and ev     ----                  */
		        ep = r-r_t;
		        ev = rdot - r_tdot;
		        /**********************************************************************************************/
		        
		        /*    --------         Calculating Desired Force                -------                */
		        Fdes = (-1)*(Kp.array()*ep.array()) - (Kv.array()*ev.array())+u1_t*zb_t.array();
		        /**********************************************************************************************/
		        
		        
		        /*      -----        Calculating u1        ----         */
		        u1_des = Fdes.dot(zb);
		        /**********************************************************************************************/
		        
		        
		        /*      --------         calculating zb des            ----           */
		        zb_des = Fdes/(Fdes.norm());
		        /**********************************************************************************************/
		        
		        
		        /*       -----     Calculating Rdes       -----                  */
		        yb_des = (zb_des.cross(xc_t))/(zb_des.cross(xc_t)).norm();
		        xb_des = yb_des.cross(zb_des);
		        Rdes<<xb_des,yb_des,zb_des;
		        /**********************************************************************************************/
		        
		        /*    ----      Calculating error in rotation (er)         -----               */
		        err = (0.5)*((Rdes.transpose()*R)-(R.transpose()*Rdes));
		        er<<err(2,1),err(0,2),err(1,0);
		        /**********************************************************************************************/
		        
		        /*    ----       Calculating angular velocity in body frame  ----    */
		        wbw  = p*xb+q*yb+r1*zb;
		        /**********************************************************************************************/
		        
		        
		        /*   -----  Calculating angular velocity in body frame at the final destination ---            */
		        wbw_t = p_t*xb_t+q_t*yb_t+r1_t*zb_t;
		        /**********************************************************************************************/
		        
		        /*    ----  Computing error in angular velocity    ----      */
		        ew = wbw-wbw_t;
		        /**********************************************************************************************/
		        
		        /*    ----   Calculating u2,u3,u4    -----     */
		        ut = (-1)*Kr*er - Kw*ew;
		        u2_des = ut(0,0);
		        u3_des = ut(1,0);
		        u4_des = ut(2,0);
		        cout<<"u1: "<<u1_des<<endl;
		        cout<<"u2: "<<u2_des<<endl;;
		        cout<<"u3: "<<u3_des<<endl;
		        cout<<"u4: "<<u4_des<<endl;

		        /**********************************************************************************************/
		        
		        /*     ----  Calculating the final rotor speeds     ---   */
		        
		      

		     
		        u<<u1_des,u2_des,u3_des,u4_des;
		        rotorspeed = cont.inverse()*u;
		        double wdes1,wdes2,wdes3,wdes4;
		        wdes1 = sqrt(abs(rotorspeed(0,0)));
		        wdes2 = sqrt(abs(rotorspeed(1,0)));
		        wdes3 = sqrt(abs(rotorspeed(2,0)));
		        wdes4 = sqrt(abs(rotorspeed(3,0)));
		      /**********************************************************************************************/


		/*   -__________________        for testing _______________ */


/**********************************************************************************************/
		cout<<"___________________________Mellinger_______________________"<<endl;
		cout<<"Fdes"<<Fdes<<endl;
        cout<<"u1_des"<<u1_des<<endl;
        cout<<"acc_t"<<acc_t<<endl;
        cout<<"t_t"<<t_t<<endl;
        cout<<"jerk_t"<<jerk_t<<endl;
        cout<<"ew"<<ew<<endl;
        cout<<"ep"<<ep<<endl;
        cout<<"ev"<<ev<<endl;
        cout<<"er"<<er<<endl;
        cout<<"thetadot_t"<<thetadot_t<<endl;
        cout<<"wdes1"<<wdes1<<endl;
        cout<<"wdes2"<<wdes2<<endl;
        cout<<"wdes3"<<wdes3<<endl;
        cout<<"wdes4"<<wdes4<<endl;
        
       
		       ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<RESL::Step>("step_service");
  RESL::Step srv;
  srv.request.w1 = wdes1;
  srv.request.w2 = wdes2;
  srv.request.w3 = wdes3;
  srv.request.w4 = wdes4;
  srv.request.N = 1;
  if (check!=5)
  {
  if (client.call(srv))
  {
  	cout<<"_______________________________Step__________________"<<endl;
    cout<<srv.response<<endl;
    check=check+1;
    controller(srv.response.position.x,srv.response.position.y,srv.response.position.z,srv.response.linear_velocity.x,srv.response.linear_velocity.y,srv.response.linear_velocity.z, srv.response.angular_position.x,srv.response.angular_position.y,srv.response.angular_position.z,srv.response.angular_velocity.x,srv.response.angular_velocity.y,srv.response.angular_velocity.z,srv.response.linear_acceleration.x,srv.response.linear_acceleration.y,srv.response.linear_acceleration.z);
    

  }
  else
  {
    ROS_ERROR("Failed to call service step");
    return;
  }
  /**********************************************************************************************/

}


		        
		  
        
    };

bool mellingerController(RESL::Mellinger::Request  &req,
         RESL::Mellinger::Response &res)
{

	
    mellinger_controller::MellingerController m;
    m.init_controller(req,res);
    
    return true;



}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mellinger_server");
    YAML::Node config = YAML::LoadFile("src/RESL/config/config.yaml");
  string servicename = config["mellinger"].as<string>();
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService(servicename, mellingerController);
  ROS_INFO("Mellinger Service Ready");
  ros::spin();

  return 0;
}
