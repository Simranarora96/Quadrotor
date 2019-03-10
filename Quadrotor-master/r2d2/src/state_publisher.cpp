#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "state_publisher");

  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  tf::TransformBroadcaster odom_broadcaster;

  double th = 0.0;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  sensor_msgs::JointState joint_state;

  ros::Rate r(30);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] ="joint1";
    joint_state.name[1] ="joint2";
    joint_state.name[2] ="joint3";
    joint_state.name[3] ="joint4";
    joint_state.name[4] ="joint5";
    joint_state.name[5] ="joint6";


    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_th = vth * dt;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base";

    odom_trans.transform.translation.x = 1;
    odom_trans.transform.translation.y = 1;
    odom_trans.transform.translation.z = 1;
    odom_trans.transform.rotation.x = 0;
    odom_trans.transform.rotation.y = 0;
    odom_trans.transform.rotation.z = 0;
    odom_trans.transform.rotation.w = 1;

    //send the transform and joint state
    odom_broadcaster.sendTransform(odom_trans);
    joint_pub.publish(joint_state);

    last_time = current_time;
    r.sleep();
  }
}
