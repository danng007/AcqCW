//
//  annFindNeighbour.hpp
//  IGLFramework
//
//  Created by Want on 14/02/2017.
//
//

#include "Eigen/Dense"
#include <ANN/ANN.h>
#include <stdio.h>
using namespace std;
namespace ann {
    tuple<Eigen::MatrixXd, Eigen::MatrixXd> annNeighbour(Eigen::MatrixXd V1, Eigen::MatrixXd V2, int annK, int pointsNumber);
    Eigen::MatrixXd annSVD(Eigen::MatrixXd V1, Eigen::MatrixXd V2,Eigen::MatrixXd originalV2);
}
