#include "rls.h"



rls::rls()
{

   // P=X.transpose()*X;
    //initialize the P matrix

    P = 1e6 * Eigen::MatrixXd::Identity(4, 4);
    P_k_1 =  Eigen::MatrixXd::Identity(4, 4);
    theta_hat = 0.01* theta_hat.setOnes(4,1);
    theta_hat_k_1 = theta_hat.setOnes(4,1);
    X_k_1.setZero(1,4);
    X.setZero(1,4);
    psi_k_1.setZero(4,1);
    psi.setZero(4,1);
    Y_k_1.setZero(1,1);




    std::cout << " P "  << std::endl   << P  <<std::endl;
    std::cout << " theta_hat "  << std::endl   << theta_hat  <<std::endl;
    std::cout << "=================================rls start==========================" <<std::endl;
}


void rls::set_regressor(){

}


//return 0.5*x*x + 0.2*x + 0.3  + (rand()%100/frac);
//Eigen::VectorXd rls::update(double x, double y){
Eigen::VectorXd rls::update(double x1, double y1, double z1, double w1,double y){
    Eigen::MatrixXd lambda = Eigen::MatrixXd::Identity(1,1);
    Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(1,1);
    Eigen::MatrixXd H;//

    X_k_1 << x1, y1 ,z1,w1;
    //X_k_1 << sin(x), cos(x) ,exp(-x),exp(-3*x);
    H= X_k_1.transpose();

    Y_k_1 << y;

//    std::cout << "X_k_1" << std::endl << X_k_1 << std::endl;
//    std::cout << "H" << std::endl << H << std::endl;
//    std::cout << "Y_k_1" << std::endl << Y_k_1 << std::endl;
//     std::cout << "psi_k_1" << std::endl <<psi_k_1 << std::endl;
    //Algorithm of the RLS
    double den =(H.transpose() * P*H)(0,0) ;
    psi_k_1 = (P*H) * (lambda + H.transpose()*P*H).inverse();
    P_k_1 = P - ((P * H * H.transpose() *P) /(den +0.99)  );
    theta_hat_k_1 = theta_hat + psi_k_1*(Y_k_1 - H.transpose()*theta_hat);

   //update
   theta_hat = theta_hat_k_1;
   P = P_k_1;


   return theta_hat;

}





