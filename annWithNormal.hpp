//
//  annWithNormal.hpp
//  IGLFramework
//
//  Created by Want on 14/02/2017.
//
//
#include "acq/normalEstimation.h"
#include "acq/decoratedCloud.h"
#include "acq/cloudManager.h"
#include "Eigen/Dense"
#include <ANN/ANN.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
using namespace std;
namespace normalAnn {
    acq::NeighboursT inserNeighbours(Eigen::MatrixXd V1,int kneighbours);
    Eigen::MatrixXd calRT(Eigen::MatrixXd V1,Eigen::MatrixXd V2,acq::NormalsT n,int minRows);
}
