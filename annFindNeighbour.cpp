//
//  annFindNeighbour.cpp
//  IGLFramework
//
//  Created by Want on 14/02/2017.
//
//

#include "annFindNeighbour.hpp"
tuple<Eigen::MatrixXd, Eigen::MatrixXd> ann::annNeighbour(Eigen::MatrixXd V1, Eigen::MatrixXd V2, int annK, int pointsNumber){
    int nPts;
    ANNpointArray	sourceArray;		// source data points array
    ANNpointArray	targetArray;		// target data points array
    ANNpoint		queryPt;				// query point
    ANNidxArray		neighbourInd;					// near neighbor indices
    ANNdistArray	neighbourDis;					// near neighbor distances
    ANNkd_tree*		kdTree;					// search tree
    ANNpoint        pt;

    //Initial variables
    pt = annAllocPt(3);
    queryPt = annAllocPt(3);
    neighbourInd = new ANNidx[annK];
    neighbourDis = new ANNdist[annK];

    nPts = 0;
    sourceArray = annAllocPts(pointsNumber, 3);
    targetArray = annAllocPts(V1.rows(), 3);
    cout<< "Sampled Points Number: " << pointsNumber<<endl;
    for (int i=0; i<pointsNumber; i++) {
        pt = sourceArray[i];
        for(int d=0; d<3;d++){
            pt[d] = V2(round(i*(V2.rows()/pointsNumber)),d); //Sample the number of points

//            pt[d] = V2(i,d);
        }
        sourceArray[i]=pt;
    }
    for (int i=0; i<V1.rows(); i++) {
        pt = targetArray[i];
        for(int d=0; d<3;d++){
            pt[d] = V1(i,d);
        }
        targetArray[i]=pt;
    }
    kdTree = new ANNkd_tree(
                            targetArray,
                            V1.rows(),
                            3
                            );
    Eigen::MatrixXd V3; //neighbour Vs
    V3.setZero(pointsNumber,3);
    Eigen::MatrixXd newV2; //neighbour Vs
    newV2.setZero(pointsNumber,3);

    for(int i=0;i <pointsNumber;i++){
        
        queryPt = sourceArray[i];
        //            cout<<"Source: " << i<<endl;
        kdTree->annkSearch(queryPt, annK, neighbourInd, neighbourDis);
        //            cout<<"target: "<<neighbourInd[0]<<" No.2: "<<neighbourInd[1]<<endl<<endl;
        pt = targetArray[neighbourInd[0]];
        for(int d=0; d<3;d++){
            V3(i,d) = pt[d];
        }
        for(int d=0; d<3;d++){
            newV2(i,d) = queryPt[d];
        }
    }
    
    return std::make_tuple(V3, newV2);
}

Eigen::MatrixXd ann::annSVD(Eigen::MatrixXd V3, Eigen::MatrixXd V2,Eigen::MatrixXd originalV2){
    Eigen::MatrixXd SVDC;
    float v3XMean = V3.col(0).mean();
    float v3YMean = V3.col(1).mean();
    float v3ZMean = V3.col(2).mean();
    Eigen::Vector3d v3Mean;
    v3Mean << v3XMean, v3YMean, v3ZMean;
    
    //V2 = T*V1;
    float v2XMean =V2.col(0).mean();
    float v2YMean =V2.col(1).mean();
    float v2ZMean =V2.col(2).mean();
    Eigen::Vector3d v2Mean;
    v2Mean << v2XMean, v2YMean, v2ZMean;
    cout << "V3rows: "<<V3.rows()<<endl<<"V2rows:"<<V2.rows()<<endl;
    SVDC = ((V3-v3Mean.replicate(1,V3.rows()).transpose()).transpose())*(V2-v2Mean.replicate(1,V2.rows()).transpose());
    cout << "The SVDC is"<<endl << SVDC << endl;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(SVDC, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd SVDU = svd.matrixU();
    Eigen::MatrixXd SVDV = svd.matrixV();
    Eigen::MatrixXd SVDR = SVDV*SVDU.transpose();
    Eigen::MatrixXd SVDT = v2Mean - SVDR*v3Mean;
    cout << "The SVDR is" << endl << SVDR << endl;
    cout << "The SVDT is" << endl << SVDT << endl;
    originalV2 = (originalV2 - SVDT.replicate(1,originalV2.rows()).transpose())*SVDR;
    
     return originalV2;
}


