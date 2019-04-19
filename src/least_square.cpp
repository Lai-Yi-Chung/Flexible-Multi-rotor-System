#include "least_square.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
Least_square::Least_square()
{
    rotation_index = 0;
    coeff_dim = 4;
    input_data_dim = 100;
    A.setZero(input_data_dim,coeff_dim);
    B.setZero(input_data_dim,1);
    X.setZero(coeff_dim,1);
}

Eigen::VectorXd Least_square :: update(Eigen::VectorXd new_input, double y){

    (rotation_index == input_data_dim-1) ? (rotation_index = 0) : (rotation_index++) ;

    for(int i = 0 ; i < coeff_dim ; i++){
        A(rotation_index,i) = new_input(i);
    }
    B(rotation_index) = y;

    X = (A.transpose() * A).inverse() * A.transpose() * B ;

    return X;

}
