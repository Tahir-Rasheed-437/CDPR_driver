// This file subscibes to a position and converts this into a joint velocity which is publish to the driver

// a callback from asking for Cartesian position

#include <ros/ros.h>
#include <stdlib.h>     /* srand, rand */
#include "sensor_msgs/JointState.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>

#include <visp/vpHomogeneousMatrix.h>
#include <br_driver/controller_class.h>

// Function to load attachment points



int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_vis");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(1);
    int number_of_cables;

    double ratio;
    nh.getParam("number_of_cables",number_of_cables);
    nh.getParam("drum_radius",ratio);

    controller_class CableRobot(nh,number_of_cables,"~",false); // initialise class without publisher



    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    vpHomogeneousMatrix wTp_estimate;
    visualization_msgs::Marker marker;

    while(ros::ok())
    {

        // Create frame
        u_int32_t shape18=visualization_msgs::Marker::CUBE;


        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;
        CableRobot.GetEstimatedPlatformTransformation(wTp_estimate); // Get Estimated Location


        //
        marker.pose.position.x = CableRobot.wTai[0][0][3];
        marker.pose.position.y = CableRobot.wTai[0][1][3];
        marker.pose.position.z = CableRobot.wTai[0][2][3];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //

        marker.header.frame_id="world";
        marker.header.stamp=ros::Time::now();
        marker.ns="basic_shapes";
        marker.id=0;

        marker.type = shape18;
        marker.action=visualization_msgs::Marker::ADD;



        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        while (marker_pub.getNumSubscribers() < 1)
        {
          if (!ros::ok())
          {
            return 0;
          }
          ROS_WARN_ONCE("Please create a subscriber to the marker");
          sleep(1);
        }
        marker_pub.publish(marker);
    }
    return 0;
}



