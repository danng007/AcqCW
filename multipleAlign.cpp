//
//  multipleAlign.cpp
//  IGLFramework
//
//  Created by Want on 16/02/2017.
//
//

#include "multipleAlign.hpp"

Eigen::MatrixXd mul::deMean(Eigen::MatrixXd V1){
    double c1 = V1.col(0).mean();
    double c2 = V1.col(1).mean();
    double c3 = V1.col(2).mean();
    Eigen::Vector3d meanV;
    int rows =V1.rows();
    meanV << c1,c2,c3;
    V1 = V1-meanV.replicate(1,rows).transpose();
    return V1;
}
Eigen::MatrixXd mul::closestC(Eigen::MatrixXd V1,Eigen::MatrixXd V2,Eigen::MatrixXd V3,Eigen::MatrixXd V4,Eigen::MatrixXd V5,int c){
    double c1[5];
    c1[0] = V1.col(0).mean();
    double c2[5];
    c2[0] = V1.col(1).mean();
    double c3[5];
    c3[0] = V1.col(2).mean();
    
    
    c1[1] = V2.col(0).mean();
    c2[1] = V2.col(1).mean();
    c3[1] = V2.col(2).mean();
    
    c1[2] = V3.col(0).mean();
    c2[2] = V3.col(1).mean();
    c3[2] = V3.col(2).mean();
    
    c1[3] = V4.col(0).mean();
    c2[3] = V4.col(1).mean();
    c3[3] = V4.col(2).mean();
  
    c1[4] = V5.col(0).mean();
    c2[4] = V5.col(1).mean();
    c3[4] = V5.col(2).mean();
    int result = 1;
    double minDis = 1000.0f;
    double currDis;
    for (int i=0; i<5; i++) {
        if (i == c-1) {
            continue;
        }
        currDis = (c1[i]-c1[c])*(c1[i]-c1[c])+(c2[i]-c2[c])*(c2[i]-c2[c])+(c3[i]-c3[c])*(c3[i]-c3[c]);
        if (currDis < minDis){
            result = i;
            minDis = currDis;
        }
    }
    switch (result) {
        case 1:
            return V1;
            break;
        case 2:
            return V2;
            break;
        case 3:
            return V3;
            break;
        case 4:
            return V4;
            break;
        case 5:
            return V5;
            break;
        default:
            break;
    }

    return V1;
}
