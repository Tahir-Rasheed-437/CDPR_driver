

#include "br_driver/controller_class.h"


#ifndef PI
#define PI 3.14159265359
#endif



controller_class::controller_class(ros::NodeHandle nh_, int number_of_cables, std::string frame_name, bool publishing_platform)
    : n(nh_),
      wTbi(number_of_cables),
      wTai(number_of_cables),
      pTbi(number_of_cables),
      aiTbi(number_of_cables),
      frame_name_(frame_name),
      publishing_platform_(publishing_platform)
{
    nbr=number_of_cables;
    wTai=get_attachment_parameters("base_attachment_points",n);
    pTbi=get_attachment_parameters("platform_attachment_points",n);
    get_initial_location("initial_platform_location",n);

    this->SetJointFlag(false);
    this->SetDesiredTransformFlag(false);
    joint_sub = n.subscribe("/joint_state", 1, &controller_class::JointSensorCallback, this);
    desired_transform_sub = n.subscribe("/tf", 1, &controller_class::DesiredFrameCallback, this);

    if(publishing_platform_)
    {
        ROS_INFO("Publishing tf");
        PublisherThread = std::thread(&controller_class::tfPublisher, this);
    }
}



void controller_class::JointSensorCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint=*msg; // joint is eqaul to the value pointed to by msg
    this->SetJointFlag(true); // Note that value has been receieved
}

void controller_class::DesiredFrameCallback(const tf2_msgs::TFMessageConstPtr& msg)
{

    for (int i = 0; i < msg->transforms.size(); ++i) {

        if(msg->transforms[i].child_frame_id=="desired_platform")
        {
            vpTranslationVector trans(msg->transforms[i].transform.translation.x,
                                      msg->transforms[i].transform.translation.y,
                                      msg->transforms[i].transform.translation.z);
            vpQuaternionVector quat(msg->transforms[i].transform.rotation.x,
                                    msg->transforms[i].transform.rotation.y,
                                    msg->transforms[i].transform.rotation.z,
                                    msg->transforms[i].transform.rotation.w);
            wTp_desired_.buildFrom(trans,quat);
            this->SetDesiredTransformFlag(true); // Note that value has been receieved
        }
        if(msg->transforms[i].child_frame_id=="estimated_platform_frame")
        {
            vpTranslationVector trans(msg->transforms[i].transform.translation.x,
                                      msg->transforms[i].transform.translation.y,
                                      msg->transforms[i].transform.translation.z);
            vpQuaternionVector quat(msg->transforms[i].transform.rotation.x,
                                    msg->transforms[i].transform.rotation.y,
                                    msg->transforms[i].transform.rotation.z,
                                    msg->transforms[i].transform.rotation.w);
            wTp_estimated_.buildFrom(trans,quat);
            this->SetEstimatedTransformFlag(true); // Note that value has been receieved
        }
    }

}


void controller_class::SetDesiredTransformFlag(bool Flag)
{
    DesiredTransformReceived=Flag;
}
void controller_class::SetEstimatedTransformFlag(bool Flag)
{
    EstimatedTransformReceived=Flag;
}

void controller_class::SetJointFlag(bool Flag)
{
    jointStateReceived=Flag;
}


void controller_class::SetPlatformFrameName(std::string frame_name)
{
    frame_name_=frame_name;
}

// Public functions to get receive flags
bool controller_class::GetJointFlag()
{
    return jointStateReceived;
}
bool controller_class::GetDesiredTransformFlag()
{
    return DesiredTransformReceived;
}
bool controller_class::GetEstimatedTransformFlag()
{
    return EstimatedTransformReceived;
}

void controller_class::GetRobotJointState(sensor_msgs::JointState& return_joint)
{

    return_joint.position.clear();
    return_joint.position.resize(joint.position.size());
    return_joint.position=joint.position;

    return_joint.velocity.clear();
    return_joint.velocity.resize(joint.velocity.size());
    return_joint.velocity=joint.velocity;

    return_joint.effort.clear();
    return_joint.effort.resize(joint.effort.size());
    return_joint.effort=joint.effort;

}

void controller_class::Stop(){
    PublisherThread.join();
}


std::vector<vpHomogeneousMatrix> controller_class::get_attachment_parameters(
        std::string param_name,ros::NodeHandle n)
{
    XmlRpc::XmlRpcValue Axml;
    n.getParam(param_name,Axml);
    std::vector<vpHomogeneousMatrix> T(Axml.size());


    for (int i = 0; i < Axml.size(); ++i) {
        double xa=Axml[i][0];     double ya=Axml[i][1];
        double za=Axml[i][2];
        T[i].buildFrom(xa/1000.,ya/1000.,za/1000,0,0,0);
    }

    return T;
}

void controller_class::get_initial_location(std::string param_name,ros::NodeHandle n)
{
    XmlRpc::XmlRpcValue Axml;
    n.getParam(param_name,Axml);
    vpRotationMatrix R;
    vpTranslationVector t;
    vpRxyzVector Phi;
    // x y z Rx Ry Rz
    Phi[0]=Axml[3];
    Phi[1]=Axml[4];
    Phi[2]=Axml[5];
    double x=Axml[0];
    double y=Axml[1];
    double z=Axml[2];
    t.buildFrom(x/1.,y/1.,z/1.);
    R.buildFrom(Phi);
    wTp_.buildFrom(t,R);
}

// Function for Tahir tests, gets the trajectory parameter from the parameter server
// The parameters are simply vectors
std::vector<double> controller_class::get_trajectory_parameter(std::string param_name)
{
    XmlRpc::XmlRpcValue Axml;
    ros::param::get(param_name,Axml);

    std::vector<double> p(Axml.size());
    for (int i = 0; i < Axml.size(); ++i) {

        double element=Axml[i];
        p[i]=(element);
    }
    return p;
}

void controller_class::printVectorDouble(std::vector<double> p,const char* intro)
{
    std::cout<<intro<<"=[ ";
    for (int i = 0; i < p.size(); ++i) {
        std::cout<<p[i]<<"  ";
    }
    std::cout<<"]"<<std::endl;
}

void controller_class::printVectorDouble(vpColVector p,const char* intro)
{
    std::cout<<intro<<"=[ ";
    for (int i = 0; i < p.getRows(); ++i) {
        std::cout<<p[i]<<"  ";
    }
    std::cout<<"]"<<std::endl;
}

void controller_class::printfM(vpHomogeneousMatrix M,const char* intro)
{
    printf(intro,"\n");
    printf("\n");
    printf("%8.3f %8.3f %8.3f %8.8f\n", M[0][0], M[0][1], M[0][2], M[0][3]);
    printf("%8.3f %8.3f %8.3f %8.8f\n", M[1][0], M[1][1], M[1][2], M[1][3]);
    printf("%8.3f %8.3f %8.3f %8.8f\n", M[2][0], M[2][1], M[2][2], M[2][3]);
    printf("%8.3f %8.3f %8.3f %8.3f\n", M[3][0], M[3][1], M[3][2], M[3][3]);
}

void controller_class::GetPlatformTransformation(vpHomogeneousMatrix& M)
{
    M=wTp_;
}
void controller_class::GetDesiredPlatformTransformation(vpHomogeneousMatrix& M)
{
    M=wTp_desired_;
}
void controller_class::GetEstimatedPlatformTransformation(vpHomogeneousMatrix& M)
{
    M=wTp_estimated_;
}

void controller_class::UpdatePlatformTransformation(vpHomogeneousMatrix M)
{
    wTp_=M;
}

void controller_class::UpdatePlatformTransformation(vpTranslationVector t,vpQuaternionVector Q)
{
    wTp_.buildFrom(t,Q);
}

void controller_class::UpdatePlatformTransformation(double x,double y,double z,double Rx, double Ry,double Rz)
{
    vpRotationMatrix R;
    vpTranslationVector t;
    vpRxyzVector Phi;
    Phi[0]=Rx;
    Phi[1]=Ry;
    Phi[2]=Rz;
    t.buildFrom(x,y,z);
    R.buildFrom(Phi);
    wTp_.buildFrom(t,R);
}



void controller_class::CartesianError(vpHomogeneousMatrix T,vpHomogeneousMatrix Td,vpColVector& Error)
{

    assert(Error.size()==6);
    vpTranslationVector Pd,P;
    vpRotationMatrix Rd,R,DiffMat;
    vpThetaUVector u;
    Td.extract(Pd);
    Td.extract(Rd);

    T.extract(P);
    T.extract(R);

    DiffMat=Rd*R.inverse(); // Rotation difference between desired and current frames

    u.buildFrom(DiffMat);
    Error[0]=Pd[0]-P[0];
    Error[1]=Pd[1]-P[1];
    Error[2]=Pd[2]-P[2];
    Error[3]=u[0];
    Error[4]=u[1];
    Error[5]=u[2];


}

double controller_class::traj_interpolator(double tf_t, int Interpolator)
{
    double rt;

    if(tf_t<0.0)
    {
        rt=0.0;
    }
    else if(tf_t<1.0)
    {
        switch(Interpolator)
        {
        case 1://linear
            rt=tf_t;
            break;

        case 2: //"cubic":
            rt=(    3*(pow((tf_t),2)))    -(2*( pow((tf_t),3)));
            break;
        case 3://"quintic":
            rt=(    10*(pow((tf_t),3)))    -(15*( pow((tf_t),4)))   + (6*(pow((tf_t),5))    );
            break;

        }
    }
    else
    {
        rt=1.0;
    }
    return rt;
}

/*************************************************************************************************************

  Interpolate between  T_initial & T_final at time t, such that
      T_initial corresponds to platform position at time 0
      T_final corresponds to platform position at time t_final
      T_desired corresponds to platform position at time t

      The initial and final speed is zero
      Interpolation is completed using polynomial


   *************************************************************************************************************/

void controller_class::CartesianTrajectoryInterpolation(vpHomogeneousMatrix T_initial,
                                                        vpHomogeneousMatrix T_final,
                                                        ros::Duration t,
                                                        ros::Duration t_final,
                                                        vpHomogeneousMatrix& T_desired,
                                                        int Interpolator)
{

    // change Rdes to Rfinal
    vpTranslationVector u;
    vpMatrix uskew(3,3);
    vpMatrix I(3,3);
    vpMatrix A(3,3);
    vpMatrix B(3,3);
    vpMatrix C(3,3);
    vpThetaUVector uAlpha;

    vpRotationMatrix Rdes,Rt,Rinit,RotuAlpha,RotuAlpha_rt;
    vpTranslationVector Pt,Pinit,Pdes;

    double nu,alpha;
    double rt,tf_t;

    tf_t=(t.toSec()/(t_final.toSec()));
    rt=traj_interpolator(tf_t,Interpolator);

    if(t.toNSec()<0) ROS_ERROR("Negative time, something is very very wrong");

    T_final.extract(Rdes);
    T_final.extract(Pdes);
    T_initial.extract(Rinit);
    T_initial.extract(Pinit);

    RotuAlpha=Rdes*Rinit.inverse(); // Takes the difference between the intial and final points of a section in oreintation
    uAlpha.buildFrom(RotuAlpha); // u theta Representation of Orientation
    alpha = sqrt(pow(uAlpha[0],2)+pow(uAlpha[1],2)+pow(uAlpha[2],2));

    if(fabs(alpha)<0.001) // Set to identity
    {
        RotuAlpha_rt.eye();
        u.set(0.0,0.0,0.0);
    }
    else
    {
        u.buildFrom(uAlpha[0]/alpha,uAlpha[1]/alpha,uAlpha[2]/alpha);
        uskew=u.skew();

        nu=rt*alpha;

        I.eye(3);
        A.eye(3,3);
        B.eye(3,3);
        C.eye(3,3);

        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                A[i][j]=u[i]*u[j]*(1-cos(nu));
                B[i][j]=I[i][j]*cos(nu);
                C[i][j]=uskew[i][j]*sin(nu);
                RotuAlpha_rt[i][j]=A[i][j]+B[i][j]+C[i][j];
            }
        }
    }

    Rt=RotuAlpha_rt*Rinit;
    Pt=(Pdes-Pinit);
    Pt=Pinit+(Pt*rt);
    T_desired.buildFrom(Pt,Rt);

}

// Calculations
/*===========================================================================================

        Each of these functions are overloaded with one using the tracked platform pose
        and a second allowing user input

  ===========================================================================================*/


// calculate_cable_length() obtain the lenght of each cable at current platform pose
std::vector<double> controller_class::calculate_cable_length()
{
    std::vector<double> cable_length(nbr);
    vpTranslationVector L;
    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform  attament position w.r.t world frame
        wTbi[i]=wTp_*pTbi[i];
        // Platform position w.r.t corresponding base

        // Extract position of base attachment points w.r.t world frame
        wTai[i].extract(wPai);
        // Extract position of platform attachment points w.r.t world frame
        wTbi[i].extract(wPbi);
        // Cable Vector
        L=wPbi-wPai;
        cable_length[i]=L.euclideanNorm();
    }
    return cable_length;
}



std::vector<double> controller_class::calculate_cable_length(vpHomogeneousMatrix wTp)
{


    std::vector<double> cable_length(nbr);
    vpTranslationVector L;
    vpTranslationVector wPai,wPbi;
    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];

        // Extract position of base attachment points w.r.t world frame
        wTai[i].extract(wPai);
        // Extract position of platform attachment points w.r.t world frame
        wTbi[i].extract(wPbi);
        // Cable Vector
        L=wPbi-wPai;
        cable_length[i]=L.euclideanNorm();
    }
    return cable_length;
}



// calculate_cable_vectors() : obtain cable vector in anchor frame at current
// platform pose
std::vector<vpTranslationVector> controller_class::calculate_cable_vectors()
{

    std::vector<vpTranslationVector> L(nbr);
    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp_*pTbi[i];
        // Platform position w.r.t corresponding base
        wTai[i].extract(wPai);
        wTbi[i].extract(wPbi);
        L[i]=wPbi-wPai;

    }
    return L;
}
// calculate_cable_vectors(vpHomogeneousMatrix wTp) : obtain cable vector in anchor frame at wTp
std::vector<vpTranslationVector> controller_class::calculate_cable_vectors(vpHomogeneousMatrix wTp)
{

    std::vector<vpTranslationVector> L(nbr);
    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];
        // Platform position w.r.t corresponding base
        wTai[i].extract(wPai);
        wTbi[i].extract(wPbi);
        L[i]=wPbi-wPai;

    }
    return L;
}

// calculate_normalized_cable_vectors() : obtain cable normalized vector in anchor frame at current
// platform pose
std::vector<vpTranslationVector> controller_class::calculate_normalized_cable_vectors()
{
    std::vector<vpTranslationVector> L(nbr);
    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp_*pTbi[i];
        // Platform position w.r.t corresponding base
        wTai[i].extract(wPai);
        wTbi[i].extract(wPbi);
        L[i]=wPbi-wPai;
        //L[i].normalize();
        for (int j = 0; j < L[i].size(); ++j) {
            L[i][j]=L[i][j]/L[i].euclideanNorm();
        }
    }
    return L;
}

// calculate_normalized_cable_vectors(vpHomogeneousMatrix wTp) : obtain cable normalized vector
// in anchor frame at wTp
std::vector<vpTranslationVector> controller_class::calculate_normalized_cable_vectors(vpHomogeneousMatrix wTp)
{   
    std::vector<vpTranslationVector> L(nbr);
    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];
        // Platform position w.r.t corresponding base
        wTai[i].extract(wPai);
        wTbi[i].extract(wPbi);
        L[i]=wPbi-wPai;
        //L[i].normalize();
        for (int j = 0; j < L[i].size(); ++j) {
            L[i][j]=L[i][j]/L[i].euclideanNorm();
        }
    }
    return L;
}



// Find joint deviation required to move to wTp_desired from current location
std::vector<double> controller_class::calculate_motor_change(vpHomogeneousMatrix wTp_desired,double ratio)
{

    std::vector<double> desired_cable_length(nbr);
    std::vector<double> current_cable_length(nbr);
    std::vector<double> q(nbr);

    for (int i = 0; i < nbr; ++i) {
        current_cable_length=calculate_cable_length(); // current_cable_length
        desired_cable_length=calculate_cable_length(wTp_desired); // current_cable_length
        q[i]=(desired_cable_length[i]-current_cable_length[i])/ratio;
    }
    return q;
}



/*
  This function calculates the jacobian matrix
  using Lamaury notation,
  THIS JACOBIAN HAS BEEN VERIFIED USING
  DIFFERNTIAL GEMOETRIC MODEL!
*/


void controller_class::calculate_jacobian(vpMatrix& J)
{

    std::vector<vpTranslationVector> L(nbr);
    std::vector<vpTranslationVector> u(nbr);
    vpRotationMatrix wRp;
    vpTranslationVector pPbi;

    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp_*pTbi[i];
        // Platform position w.r.t corresponding base
        wTai[i].extract(wPai);
        wTbi[i].extract(wPbi);
        L[i]=wPbi-wPai;
        //L[i].normalize();
        for (int j = 0; j < L[i].size(); ++j) {
            L[i][j]=L[i][j]/L[i].euclideanNorm();
        }
    }


    wTp_.extract(wRp);



    for (int i = 0; i < nbr; ++i) {
        pTbi[i].extract(pPbi);
        u[i] = vpTranslationVector::cross(wRp*pPbi,L[i]);

        for (int j = 0; j < 6; ++j) {

            // First 3 rows = normalized length
            // Last three rows equal u vector
            if(j<3)
            {
                J[i][j]=L[i][j];
            }
            else
            {
                J[i][j]=u[i][j-3];
            }
        }
    }
}

/*
  This function calculates the reduced jacobian matrix
  from rows in p
  for example for six rows should be p=[0,1,2,3,4,5]

*/
void controller_class::calculate_reduced_jacobian(vpMatrix J,vpMatrix& Jred,std::vector<int> p){
  // J is an 8 \times 6 matrix

    if(p.size()!=6) Jred.resize(p.size(),6); // by default we assume 6 times 6

    for (int rows = 0; rows < p.size(); ++rows) {
        for (int cols = 0; cols < 6; ++cols) {
            Jred[rows][cols]=J[p[rows]][cols];
        }
    }
}


/*
  Caluclates the inverse jacobian robot of the system @ current platform pose
  NOTE: This is the transpose of the inverse jacobian matrix

  W=J^{T}

  W \tau = f;

  */





void controller_class::calculate_inv_jacobian(vpMatrix& W)
{

    std::vector<vpTranslationVector> L;
    std::vector<vpTranslationVector> u(nbr);
    vpRotationMatrix wRp;
    vpTranslationVector pPbi;
    L=calculate_normalized_cable_vectors();
    wTp_.extract(wRp);



    for (int i = 0; i < nbr; ++i) {
        pTbi[i].extract(pPbi);
        u[i] = vpTranslationVector::cross( wRp*pPbi,L[i]);

        for (int j = 0; j < 6; ++j) {

            // First 3 rows = normalized length
            // Last three rows equal u vector
            j<=2 ? W[j][i]=L[i][j] : W[j][i]=u[i][j-3];

        }
    }

}

/*
  Caluclates the inverse jacobian robot of the system @ wTp
  
  */
void controller_class::calculate_inv_jacobian(vpHomogeneousMatrix wTp,vpMatrix& W)
{

    std::vector<vpTranslationVector> L;
    std::vector<vpTranslationVector> u(nbr);
    vpRotationMatrix wRp;
    vpTranslationVector pPbi;
    L=calculate_normalized_cable_vectors(wTp);
    wTp.extract(wRp);



    for (int i = 0; i < nbr; ++i) {
        pTbi[i].extract(pPbi);
        u[i] = vpTranslationVector::cross(wRp*pPbi,L[i]);

        for (int j = 0; j < 6; ++j) {

            // First 3 rows = normalized length
            // Last three rows equal u vector
            j<=2 ? W[j][i]=L[i][j] : W[j][i]=u[i][j-3];

        }
    }
    
}


/*
 This function extracts the matrix necessary to transform angular velocity to quaternion dot

  */
void controller_class::convert_omega_to_quaternion_dot(vpHomogeneousMatrix wTp,vpColVector omega,vpColVector& quaternion_dot)
{
    vpQuaternionVector Q;
    vpMatrix C(4,3);
    wTp.extract(Q);
    double Q1=Q.w();
    double Q2=Q.x();
    double Q3=Q.y();
    double Q4=Q.z();


    // Eq. 5.69 Khalil
    C[0][0]=-Q2; C[0][1]=-Q3; C[0][2]=-Q4;
    C[1][0]= Q1; C[1][1]= Q4; C[1][2]=-Q3;
    C[2][0]=-Q4; C[2][1]= Q1; C[2][2]= Q2;
    C[3][0]= Q3; C[3][1]=-Q2; C[3][2]= Q1;

    C=C*0.5;

    quaternion_dot=C*omega;

}

void controller_class::convert_omega_to_quaternion_dot(vpHomogeneousMatrix wTp,double omega_x,double omega_y,double omega_z,vpColVector& quaternion_dot)
{
    vpQuaternionVector Q;
    vpMatrix C(4,3);
    vpColVector omega(3);
    omega[0]=omega_x;
    omega[1]=omega_y;
    omega[2]=omega_z;
    wTp.extract(Q);
    double Q1=Q.w();
    double Q2=Q.x();
    double Q3=Q.y();
    double Q4=Q.z();


    // Eq. 5.69 Khalil
    C[0][0]=-Q2; C[0][1]=-Q3; C[0][2]=-Q4;
    C[1][0]= Q1; C[1][1]= Q4; C[1][2]=-Q3;
    C[2][0]=-Q4; C[2][1]= Q1; C[2][2]= Q2;
    C[3][0]= Q3; C[3][1]=-Q2; C[3][2]= Q1;

    C=C*0.5;

    quaternion_dot=C*omega;

}
/*

 This function finds wTp t+1 from wTpt and by integrating a twist

*/
void controller_class::integrate_twist(vpHomogeneousMatrix& wTp,
                                   vpColVector V)
{

    ROS_ASSERT(V.getRows()==6);
    vpColVector Quaternion_dot(4); // convert
    vpTranslationVector w_P_p;
    vpQuaternionVector w_Quaternion_p;

    // convert omega to quaternion dot
    convert_omega_to_quaternion_dot(wTp,V[3],V[4],V[5],Quaternion_dot);
    // get current quaternion
    wTp.extract(w_Quaternion_p);
    wTp.extract(w_P_p);

    // inetgrate
    w_Quaternion_p.buildFrom(w_Quaternion_p.x()+Quaternion_dot[1],
            w_Quaternion_p.y()+Quaternion_dot[2],
            w_Quaternion_p.z()+Quaternion_dot[3],
            w_Quaternion_p.w()+Quaternion_dot[0]);
    w_Quaternion_p.normalize();
    w_P_p.buildFrom(w_P_p[0]+V[0],
            w_P_p[1]+V[1],
            w_P_p[2]+V[2]);

    wTp.buildFrom(w_P_p,w_Quaternion_p);
}

double controller_class::get_vector_error(vpColVector p,vpColVector p1){
    ROS_ASSERT(p.getRows()==p1.getRows());
    double sum;
       for (int i = 0; i < p.size(); ++i) {
           sum=sum+pow(p[i]-p1[i],2);
       }
    return pow(sum,0.5);

}

double controller_class::get_vector_error(std::vector<double> p,std::vector<double> p1){
    ROS_ASSERT(p.size()==p1.size());
    double sum;
       for (int i = 0; i < p.size(); ++i) {
           sum=sum+pow(p[i]-p1[i],2);
       }
    return pow(sum,0.5);

}


/*
======================================================================

void security_publisher();
Runs in a seperate thread to publish the data.

======================================================================
*///
void controller_class::tfPublisher()
{

    ros::Rate r(30);
    geometry_msgs::TransformStamped T,Ta,Tb;
    vpQuaternionVector Q;
    vpTranslationVector t;
    static tf::TransformBroadcaster br;
    ros::Publisher cable_pub=n.advertise< br_driver::robot_cables_msgs >("cable_state",1);
    std::vector<vpTranslationVector> L;
    br_driver::robot_cables_msgs L_msg;
    L_msg.cable.resize(nbr);

    for (int i = 0; i < nbr; ++i) {
        L_msg.cable[i].cable_vector.resize(3);
    }




    T.header.frame_id="world";
    T.child_frame_id=frame_name_;

    std::cout<<"frame_name_= "<<frame_name_<<std::endl;
    ros::Time t_n;
    int Initial=1;

    while(ros::ok())
    {
        t_n=ros::Time::now();
        T.header.stamp.sec=t_n.sec;
        T.header.stamp.nsec=t_n.nsec;

        wTp_.extract(Q);
        wTp_.extract(t);
        T.transform.translation.x=t[0];
        T.transform.translation.y=t[1];
        T.transform.translation.z=t[2];
        T.transform.rotation.w=Q.w();
        T.transform.rotation.x=Q.x();
        T.transform.rotation.y=Q.y();
        T.transform.rotation.z=Q.z();
        br.sendTransform(T);


        if(!Initial) // Can send on initial iteration
        {           // as platform is not existing
            for (int var = 0; var < nbr; ++var) {
                Tb.header.frame_id=frame_name_;
                Tb.header.stamp.sec=t_n.sec;
                Tb.header.stamp.nsec=t_n.nsec;
                std::string child_name="platform_attachment_"+
                        boost::lexical_cast<std::string>(var+1);
                Tb.child_frame_id=child_name;
                pTbi[var].extract(Q);
                pTbi[var].extract(t);
                Tb.transform.translation.x=t[0];
                Tb.transform.translation.y=t[1];
                Tb.transform.translation.z=t[2];
                Tb.transform.rotation.w=Q.w();
                Tb.transform.rotation.x=Q.x();
                Tb.transform.rotation.y=Q.y();
                Tb.transform.rotation.z=Q.z();
                br.sendTransform(Tb);
            }
        }
        else
        {
            Initial=0;
        }

        for (int var = 0; var < nbr; ++var) {
            Ta.header.frame_id="world";
            std::string child_name="base_attachment_"+
                    boost::lexical_cast<std::string>(var+1);
            Ta.child_frame_id=child_name;
            Ta.header.stamp.sec=t_n.sec;
            Ta.header.stamp.nsec=t_n.nsec;
            wTai[var].extract(Q);
            wTai[var].extract(t);
            Ta.transform.translation.x=t[0];
            Ta.transform.translation.y=t[1];
            Ta.transform.translation.z=t[2];
            Ta.transform.rotation.w=Q.w();
            Ta.transform.rotation.x=Q.x();
            Ta.transform.rotation.y=Q.y();
            Ta.transform.rotation.z=Q.z();
            br.sendTransform(Ta);
        }

        L=calculate_cable_vectors(wTp_);
        for (int i = 0; i < nbr; ++i) {
            L_msg.cable[i].cable_number=i;
            L_msg.cable[i].cable_length=L[i].euclideanNorm();
            for (int j = 0; j < 3; ++j) {
                L_msg.cable[i].cable_vector[j]=L[i][j];
            }
        }
        cable_pub.publish(L_msg);
        r.sleep();
        ros::spinOnce();

    }
}
