#include "id.h"

//constructer ID
ID::ID(Robot robot):robot_(robot){

    NB= robot.getDofs(); // No. of link in the robot.
    
}

// main code for Recursive newton Euler algorithm
Eigen::VectorXd ID::rnea(const Eigen::VectorXd &q, const Eigen::VectorXd &qd, const Eigen::VectorXd &qdd){
    std::cout<< "RNEA code starts here.."<<std::endl;
    std::vector<Eigen::VectorXd> v(NB); // spatial velocity vector(6*NB)
    std::vector<Eigen::VectorXd> a(NB);// spatial acceleration vector(6*NB)
    std::vector<Eigen::VectorXd> f(NB);// spatial force vector(6*NB)
    std::vector<Eigen::Matrix<double, 6, 6>> Xup(NB);//
    Eigen::VectorXd a_grav;// acceleration due to gravity vector.
    a_grav.resize(6);
    a_grav<<0,0,0,0,0,-9.81;
   
    Eigen::VectorXd S;//Joint's motion subspace.
    S.resize(6);
    std::vector<Link> links;
    robot_.getLinks(links);
    theta_.resize(NB);//DH  Param
    d_.resize(NB);//DH  Param
    a_.resize(NB);//DH  Param
    alpha_.resize(NB);//DH  Param
    f.resize(NB);
  
    for(short int i=0; i<NB; i++){

        Eigen::Matrix<double, 6, 6> XJ;//Joint transform
        Eigen::Matrix<double, 6, 6> XT;
        Eigen::VectorXd I;//spatial inertia
        Eigen::Matrix<double, 6, 6> Ii;//inertia along the joint.
        Eigen::Matrix<double,3,3> Icm;//Inertia along Center of Mass
        Eigen::VectorXd com; // COM coordinate.
        Eigen::VectorXd vj;
        double m;// mass of the link.
        XJ.setZero();
        XT.setZero();
        S.setZero();

        jcalc(q[i], XJ, S);// For joint transform and motion subpace calculation.
        
        std::vector<Link> link;
        DHParams dh_params;// DH Params of the link.
        DynamicParameters dynamic_parameters;// Dynamic Parametrs of the link.
        robot_.getLinks(link);
        link[i].getDHParams(dh_params);// gets the DH param of ith link.
        link[i].getDynamicParameters(dynamic_parameters);// gets the Dynamic param of ith link.

        theta_(i) = dh_params.theta;
        d_(i) = dh_params.d;
        a_(i) = dh_params.a;
        alpha_(i) = dh_params.alpha;

        xtcalc(a_[i],d_[i],alpha_[i],XT);// To calculate XT using xtcalc function
     
        Xup[i]=XJ*XT; // XUP
        vj = S* qd[i];// spatial velicty for link i.
        
        //if i==0, their is no parent link.
        if (i == 0) {
            v[i] = vj ;
            a[i] = Xup[i] *(- a_grav) + S * qdd[i];
        } else {
            v[i] = Xup[i] * v[i-1] + vj;
            a[i] = Xup[i] * a[i-1] + S * qdd[i] + crm(v[i]) * vj;
        }
        
        double l;
        l=sqrt(a_[i]*a_[i]+d_[i]*d_[i]);
        I=dynamic_parameters.inertia;
        m=dynamic_parameters.mass;
        com=dynamic_parameters.com;

        Icm<<I[0],0,0,
            0,I[3],0,
            0,0,I[5];
        
        Icm=m*(l*l)*Icm;//Inertia along center of mass
        Ii = mci(m,com,Icm);// spatial Inertia
        f[i] = Ii * a[i] + crf(v[i]) * Ii * v[i];// Spatial force calculation
    }
    tau_.resize(NB);
    
    for (int i = NB-1 ; i >= 0; i--) {
        tau_[i] = S.transpose()* f[i];
        if (i-1 != -1)
            f[i-1] += Xup[i].transpose() * f[i];
    }
    return tau_;
}

//jcalc function
void ID::jcalc(const double &qi, Eigen::Matrix<double, 6, 6>& XJ, Eigen::VectorXd& S){
    double c = cos(qi);
    double s = sin(qi);
    //as there is only revolute joint in this model, XJ will be equal to XrotZ.
    XJ << c, s, 0.0, 0.0, 0.0, 0.0,
          -s, c, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, c, s, 0.0,
          0.0, 0.0, 0.0, -s, c, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0;  
    S<<0,0,1,0,0,0;// As only rotational joint.
}

//ctalc funtion for calculation of XT
void ID::xtcalc(const double ai, const double di, const double alphai,  Eigen::Matrix<double, 6, 6>& XT ){
    Eigen::Matrix<double, 6, 6> XrotX;
    Eigen::Matrix<double, 6, 6> Xtrans;
    double c = cos(alphai);
    double s = sin(alphai);
    XrotX<<1,0,0,0,0,0,
           0,c,s,0,0,0,
           0,-s,c,0,0,0,
           0,0,0,1,0,0,
           0,0,0,0,c,s,
           0,0,0,0,-s,c;

    Xtrans<<1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,1,0,0,0,
            0,di,0,1,0,0,
            -di,0,ai,0,1,0,
            0,-ai,0,0,0,1;
    
    XT=XrotX*Xtrans;
}

// crm  spatial cross-product operator (motion)
Eigen::Matrix<double, 6, 6> ID::crm(const Eigen::VectorXd& v) {
    Eigen::Matrix<double, 6, 6> vcross;
    vcross << 0,    -v(2),  v(1),   0,     0,     0,
              v(2),  0,    -v(0),   0,     0,     0,
             -v(1),  v(0),  0,      0,     0,     0,
              0,    -v(5),  v(4),   0,    -v(2),  v(1),
              v(5),  0,    -v(3),   v(2),  0,    -v(0),
             -v(4),  v(3),  0,     -v(1),  v(0),  0;
    return vcross;
}

// crf spatial cross-product operator (force).
Eigen::Matrix<double, 6, 6> ID::crf(const Eigen::VectorXd& v) {
    return -crm(v).transpose();
}

//mcI  spatial rigid-body inertia from mass, CoM and rotational inertia.
// mcI(m,c,I) calculates the spatial inertia matrix of a rigid body from its
//mass, centre of mass (3D vector) and rotational inertia (3x3 matrix)
// about its centre of mass.

Eigen::Matrix<double, 6, 6> ID::mci(double m, const Eigen::VectorXd& c, const Eigen::Matrix<double, 3,3>& I) {
    
    Eigen::Matrix<double,3,3> C;
    C <<   0,    -c(2),  c(1),
           c(2),  0,    -c(0),
          -c(1),  c(0),  0;

    Eigen::Matrix<double, 6, 6> rbi;
    rbi.block<3, 3>(0, 0) = I + m * C * C.transpose();
    rbi.block<3, 3>(0, 3) = m * C;
    rbi.block<3, 3>(3, 0) = m * C.transpose();
    rbi.block<3, 3>(3, 3) = m * Eigen::Matrix3d::Identity();
    return rbi;
}

ID::~ID(){} //Destructor ID
