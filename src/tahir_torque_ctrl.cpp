// This file subscibes to a position and converts this into a joint velocity which is publish to the driver

// a callback from asking for Cartesian position

#include <ros/ros.h>
#include <stdlib.h>     /* srand, rand */
#include "sensor_msgs/JointState.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visp/vpHomogeneousMatrix.h>

#include <br_driver/CartesianTrajectory.h>
#include <br_driver/CartesianTrajectoryPoint.h>

#include <br_driver/controller_class.h>


// Function to load attachment points


int main(int argc, char **argv) {
    ros::init(argc, argv, "CartesianController");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(50);
    int number_of_cables;

    double ratio;
    nh.getParam("number_of_cables",number_of_cables);
    nh.getParam("drum_radius",ratio);

    controller_class CableRobot(nh,number_of_cables,"~",false); // initialise class without publisher

    std::vector<double> tau1;
    std::vector<double> tau2;
    std::vector<double> tau3;
    std::vector<double> tau4;
    std::vector<double> traj_time;
 //   tau1=CableRobot.get_trajectory_parameter("tension1");
  //  tau2=CableRobot.get_trajectory_parameter("tension2");
   // tau3=CableRobot.get_trajectory_parameter("tension3");
   // tau4=CableRobot.get_trajectory_parameter("tension4");
    traj_time=CableRobot.get_trajectory_parameter("traj_time");

    while(ros::ok())
    {
        for (int i = 0; i < tau1.size(); ++i) {
         //   ROS_INFO("Tau=[ %f, %f,%f,%f], Time=%f",tau1[i],tau2[i],tau3[i],tau4[i],1.2);
            ROS_INFO("traj_time=[ %f",traj_time[i]);
        }

return 0;
    }
    return 0;
}



