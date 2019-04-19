#ifndef LEAST_SQUARE_H
#define LEAST_SQUARE_H
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <stdio.h>
class Least_square
{
public:
  Least_square();

  int coeff_dim;
  int input_data_dim;
  int rotation_index;
  Eigen::VectorXd update(Eigen::VectorXd new_data, double y);
private:

  Eigen::MatrixXd A;
  Eigen::VectorXd X, B;
};

#endif // LEAST_SQUARE_H
