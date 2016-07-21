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

// The recursive method to obtain all combinations
// given by http://ideone.com/78jkV

#define SET_SIZE 8

int set[] = {0,1,2,3,4,5,6,7};
std::vector<std::vector<int> > all_combinations;
const int tuple_size = 6;

void recursive_comb(int step_val, int array_index, std::vector<int> tuple)
{
    if (step_val == 0)
    {
        all_combinations.push_back(tuple); //<==We have the final combination
        return;
    }

    for (int i = array_index; i < SET_SIZE; i++)
    {
        tuple.push_back(set[i]);
        recursive_comb(step_val - 1, i + 1, tuple);
        tuple.pop_back();
    }

    return;
}

void init_combinations()
{
    std::vector<int> tuple;
    tuple.reserve(tuple_size); //avoids needless allocations
    recursive_comb(tuple_size, 0, tuple);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(50);
    int number_of_cables;
    double ratio;//=0.00034906585039886593;

    init_combinations(); // Generate all combinations

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

    vpHomogeneousMatrix wTp_initial(trans,quat);
    vpHomogeneousMatrix wTp_desired(trans,quat);
    vpHomogeneousMatrix wTp_initial2(trans,quat);
    double delta=0.0;
    int option;
    if(argc<3)
    {
        ROS_ERROR("Requires two input arguments axis and value");
        return -1;

    }
    else
    {
        sscanf(argv[1],"%d",&option);
        sscanf(argv[2],"%lf",&delta);
    }
    vpColVector dX(6);
    vpColVector wVp(6);
    wVp[option]=delta;

    CableRobot.integrate_twist(wTp_desired,wVp);

    // Obtained wTp_desired


    sensor_msgs::JointState initial_joint_state,desired_joint_state;
    initial_joint_state.position.resize(number_of_cables);
    desired_joint_state.position.resize(number_of_cables);

    std::vector<double> initial_cable_vector(number_of_cables);
    std::vector<double> desired_cable_vector(number_of_cables);






    CableRobot.printfM(wTp_initial,"wTp_initial");
    CableRobot.printfM(wTp_initial2,"wTp_initial2");
    CableRobot.printfM(wTp_desired,"wTp_desired");

    vpMatrix J(6,8); // jacobian matrix
    vpMatrix J_lau(8,6); // jacobian matrix
    vpMatrix J_red(6,6); // jacobian matrix
    std::vector<int> red_rows(6);

    vpColVector dq(number_of_cables);
    vpColVector dl(number_of_cables);
    vpColVector dl_check(number_of_cables);
    vpColVector dl_check2(number_of_cables);
    vpColVector dl_red(6);



    double error_integration;
    vpColVector Error(6);
    double tol=0.02; // tolerance is in metres
    bool debug=true;

    CableRobot.UpdatePlatformTransformation(wTp_initial);
    double error;
    while(!CableRobot.GetJointFlag()){
        r.sleep();
    }


    // Start the checks
    initial_cable_vector=CableRobot.calculate_cable_length(wTp_initial); // current_cable_length
    desired_cable_vector=CableRobot.calculate_cable_length(wTp_desired); // current_cable_length

    for (int i = 0; i < number_of_cables; ++i) {
        dq[i]=(desired_cable_vector[i]-initial_cable_vector[i])/ratio;
        dl[i]=(desired_cable_vector[i]-initial_cable_vector[i]);
    }
    CableRobot.calculate_jacobian(J_lau);
    dX=(J_lau.pseudoInverse())*dl; // find velocity vx vy vz wx wy wz
    std::cout<<"dX.euclideanNorm()"<<dX.euclideanNorm()<<std::endl;
    std::cout<<"wVp.euclideanNorm()"<<wVp.euclideanNorm()<<std::endl;

    dl_check=J_lau*dX;

    ROS_INFO("\e[1mCHECK 0:\e[0m dl from wVp and dl by Jacobian");
    dl_check2=J_lau*wVp;
    CableRobot.printVectorDouble(dl_check2,"dl_check2 = ");

    ROS_INFO("\e[1mCHECK 1:\e[0m dl by derivative is equal to dl by Jacobian");
    CableRobot.printVectorDouble(dl,"dl = ");
    CableRobot.printVectorDouble(dl_check,"dl_check = ");
    error=CableRobot.get_vector_error(dl,dl_check);
    std::cout<<"Inetgration error= "<<error<<std::endl;


    ROS_INFO("\e[1mCHECK 2:\e[0m Input dX is equal to dX by Jacobian");
    CableRobot.printVectorDouble(wVp,"wVp input= ");
    CableRobot.printVectorDouble(dX,"dX by Jacobian= ");
    error_integration=CableRobot.get_vector_error(dX,wVp);
    std::cout<<"Inetgration error= "<<error_integration<<std::endl;

    ROS_INFO("\e[1mCHECK 3:\e[0m Desired wTp equal wTp_initial2 +dX");
    CableRobot.integrate_twist(wTp_initial2,dX);
    CableRobot.CartesianError(wTp_initial2,wTp_desired,Error);
    std::cout<<"Inetgration error= "<<Error.euclideanNorm()<<std::endl;

    ROS_INFO("\e[1mCHECK 4:\e[0m Calculating dl from new location");
    desired_cable_vector=CableRobot.calculate_cable_length(wTp_initial2); // current_cable_length
    for (int i = 0; i < number_of_cables; ++i) {
        dq[i]=(desired_cable_vector[i]-initial_cable_vector[i])/ratio;
        dl[i]=(desired_cable_vector[i]-initial_cable_vector[i]);
    }
    CableRobot.printVectorDouble(dl,"After integration dl = ");
    error=CableRobot.get_vector_error(dl,dl_check);
    std::cout<<"Inetgration error= "<<error<<std::endl;


    ROS_INFO("\e[1m Starting submatrix checks \e[0m ");

    // -------------------- start of for loops ---------------------//
//    for (int i = 0; i < all_combinations.size(); ++i) {
//        vpColVector dX_red(6);
//        red_rows=all_combinations[i];

//        CableRobot.calculate_reduced_jacobian(J_lau,J_red,red_rows);
//        //  J_red.print(std::cout,6,"J_red");

//        //        // Calculated dl reduced
//        for (int var = 0; var < 6; ++var) {
//            dl_red[var]=dl[red_rows[var]];
//        }

//        //     //   J_lau.print(std::cout,6,"J_lau");
//        //      //  ROS_INFO_COND(debug,"Calculate Dx actual");
//        dX_red=(J_red.inverseByQR())*dl_red; // find velocity vx vy vz wx wy wz

//        if(CableRobot.get_vector_error(dX_red,wVp)<error)
//        {
//        error_integration=CableRobot.get_vector_error(dX_red,wVp);
//        }
//            std::cout<<"Min Inetgration error= "<<error_integration<<std::endl<<"Current Error"<<CableRobot.get_vector_error(dX_red,wVp)<<std::endl;
//        }

        CableRobot.Stop();
        return 0;
    }



