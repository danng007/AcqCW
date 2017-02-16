//
//  multipleAlign.hpp
//  IGLFramework
//
//  Created by Want on 16/02/2017.
//
//

#include "Eigen/Dense"
#include <ANN/ANN.h>
#include <stdio.h>
using namespace std;
namespace mul {
    Eigen::MatrixXd deMean(Eigen::MatrixXd V1);
    Eigen::MatrixXd closestC(Eigen::MatrixXd V1,Eigen::MatrixXd V2,Eigen::MatrixXd V3,Eigen::MatrixXd V4,Eigen::MatrixXd V5,int c);
   
}
