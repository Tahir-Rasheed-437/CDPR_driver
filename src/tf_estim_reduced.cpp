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

// Node which completes the odmetry it assumes that robot is functioning perfectly and
// all constraints have been respected.


int main(int argc, char **argv) {

    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(100);
    int number_of_cables;
    double ratio;//=0.00034906585039886593;
    nh.getParam("number_of_cables",number_of_cables);
    nh.getParam("drum_radius",ratio);
    std::string frame_name="estimated_platform_frame";
    controller_class CableRobot(nh,number_of_cables,frame_name);

    tf::TransformListener tflistener;
    tf::StampedTransform tfstam;

    ros::Time now = ros::Time(0);

    tflistener.waitForTransform("world",frame_name,
                                now, ros::Duration(4.0));

    tflistener.lookupTransform("world",frame_name,
                               now,tfstam);

    vpTranslationVector trans(tfstam.getOrigin().getX(),
                              tfstam.getOrigin().getY(),
                              tfstam.getOrigin().getZ());
    vpQuaternionVector quat(tfstam.getRotation().getX(),
                            tfstam.getRotation().getY(),
                            tfstam.getRotation().getZ(),
                            tfstam.getRotation().getW());

    sensor_msgs::JointState current_joint_state,last_joint_state,initial_joint_state;
    last_joint_state.position.resize(number_of_cables);
    current_joint_state.position.resize(number_of_cables);
    initial_joint_state.position.resize(number_of_cables);
    std::vector<double> l(number_of_cables);
    std::vector<double> l_test(number_of_cables);
    std::vector<double> l_last(number_of_cables);
    vpHomogeneousMatrix wTp(trans,quat);
    vpHomogeneousMatrix wTp_last(trans,quat);
    vpHomogeneousMatrix wdiffp;
    vpMatrix W(6,8); // -inverse transpose of jacobian matrix
    vpMatrix WT(8,6); // inverse jacobian matrix
    vpMatrix J(6,8); // jacobian matrix
    vpMatrix J_lau(8,6); // jacobian matrix
    vpMatrix J_red(6,6); // jacobian matrix
    std::vector<int> red_rows(6);
    vpTranslationVector w_P_p;
    vpQuaternionVector w_Quaternion_p,w_Quat_p_diff;
    vpColVector Quaternion_dot(4); // convert
    vpColVector dq(number_of_cables);
    vpColVector dl(number_of_cables);
    vpColVector dl_check(number_of_cables);
    vpColVector dX(6);

    CableRobot.UpdatePlatformTransformation(wTp);
    bool InitialStep=true;
    double tol=0.01; // tolerance is in metres

    bool debug=true;


    while(!CableRobot.GetJointFlag()){
        r.sleep();
    }

    CableRobot.UpdatePlatformTransformation(wTp);
    l=CableRobot.calculate_cable_length();
    l_test=l;
    l_last=l;

    CableRobot.GetRobotJointState(current_joint_state);
    initial_joint_state=current_joint_state;
    last_joint_state=current_joint_state;

    while(ros::ok())
    {
        ROS_INFO_COND(debug,"Updating platform");
        CableRobot.UpdatePlatformTransformation(wTp);
        ROS_INFO_COND(debug,"Calculate cable length");
        l=CableRobot.calculate_cable_length();




        ROS_INFO_COND(debug,"Calculate joint state");
        CableRobot.GetRobotJointState(current_joint_state);
        // Obtain how much the joint has changed
        ROS_INFO_COND(debug,"Calculate delta q");
        for (int i = 0; i < number_of_cables; ++i)
            dq[i]=current_joint_state.position[i]-last_joint_state.position[i];
        ROS_INFO_COND(debug,"Calculate delta l");
        dl=dq*ratio; // converts from rad to m
        if(dl.euclideanNorm()>tol)
            ROS_FATAL("Norm dl is large %f. Linearization may not be valid",
                      dl.euclideanNorm());
        ROS_INFO_COND(debug,"Calculate Jacobian");
        CableRobot.calculate_jacobian(J_lau);

        // Find all reduced jacobian matrices
          // -------------------- start of for loops ---------------------//
        CableRobot.calculate_reduced_jacobian(J_lau,J_red,red_rows);

        ROS_INFO_COND(debug,"Calculate Dx");
        dX=(J_red.inverseByQR())*dl; // find velocity vx vy vz wx wy wz

        ROS_INFO_COND(debug,"Integrate Velocity");
        CableRobot.convert_omega_to_quaternion_dot(
                    wTp,dX[3],dX[4],dX[5],Quaternion_dot);

        wTp.extract(w_Quaternion_p);
        wTp.extract(w_P_p);

        w_Quaternion_p.buildFrom(w_Quaternion_p.x()+Quaternion_dot[1],
                w_Quaternion_p.y()+Quaternion_dot[2],
                w_Quaternion_p.z()+Quaternion_dot[3],
                w_Quaternion_p.w()+Quaternion_dot[0]);

        w_Quaternion_p.normalize();

        w_P_p.buildFrom(w_P_p[0]+dX[0],
                w_P_p[1]+dX[1],
                w_P_p[2]+dX[2]);

        wTp.buildFrom(w_P_p,w_Quaternion_p);

        CableRobot.printfM(wTp,"wTp by integration= ");

        wTp_last=wTp;
        l_test=CableRobot.calculate_cable_length(wTp);
        for (int i = 0; i < number_of_cables; ++i)
            dl[i]=l_test[i]-l[i];
        CableRobot.printVectorDouble(dl,"dl by integration");

        // -------------------- End of for loops ---------------------//


        last_joint_state=current_joint_state; // update last position
        l_last=l;
        ROS_INFO("==================================================");
        r.sleep();

    }
CableRobot.Stop();
return 0;
}



