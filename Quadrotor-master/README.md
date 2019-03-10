# Quadrotor

This Repository Contains Two folders:
1) quadrotor
2) R2d2

quadrotor: Contains the mellinger, step and reset controller.
R2d2: Contains the urdf file for the quadcopter.


# DEPENDENCIES:

        1) Ubuntu 16.0.4
        
        2) Ros-Kinetic
        
        3) Latest Eigen Package. Refer for link for installation and wiki - http://wiki.ros.org/eigen3
        
        4) The Yaml should be installed automatically if not already present in the ros-kinetic package - 
            by running the catkin_make command.
            
        5) This project assumes you have a working catkin workspace, ros-kinetic environment and rviz.
        
       
# INSTALLING THE PROJECT:
        
        Clone the repostory in the the src folder of your catking workspace.


# RUNNING THE PROJECT:

        RESL:
        
             Run the following commands to start the mellinger, step and reset service.
             Open the terminal and run.
                $ roscore
                
             Open another terminal and navigate to your catkin workspace.
                $ source devel/setup.bash
                $ catkin_make
                $ rosrun quadrotor step_service (This starts the step service node).
                
             Open another terminal and navigate to your catkin workspace.
                $ source devel/setup.bash
                $ rosrun quadrotor reset_service (This starts the reset service node).
                
             Open another terminal and navigate to your catkin workspace.
                $ source devel/setup.bash
                $ rosrun quadrotor mellinger_service (This starts the mellinger service node).
                
             The respective services have clients associated with them to call them individually.
             <-> step_client
             <-> reset_client
             <-> mellinger_client
             These can be used to test the services and their functioning. 
             A more detailed explaination about 
             the working can be found in the report attached.
             
         R2d2:
         
             Run the following commands to run the urdf in rviz.
             Open the terminal and run.
             $ roscore
             
             Open another terminal and navigate to your catkin workspace.
             $ source setup devel/setup.bash
             $ catkin_make
             $ rosrun r2d2 state_publisher (This starts the state publisher which 
             publishes joint states to the robot model in rviz)
             
             Open another terminal and navigate to the r2d2 directory.
             $ roslaunch display.launch
             
             Open another terminal and run.
             $ rosrun rviz rviz
             
             This will start the RVIZ. 
              - Change the fixed frame to odom.
              - Click on Add and add a robot model.
              
              This will show the robot model. Refer to report for detail
             

        
