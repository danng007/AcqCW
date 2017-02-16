//
//  annWithNormal.cpp
//  IGLFramework
//
//  Created by Want on 14/02/2017.
//
//

#include "annWithNormal.hpp"

Eigen::MatrixXd normalAnn::calRT(Eigen::MatrixXd V2,Eigen::MatrixXd V1,acq::NormalsT n,int minRows){
    Eigen::MatrixXd x;
    x.setZero(6,1);
    Eigen::MatrixXd A;
    A.setZero(minRows,6);
    Eigen::MatrixXd AP;
    
    Eigen::MatrixXd b;
    b.setZero(minRows,1);
    Eigen::MatrixXd minV1;
    minV1.setZero(minRows,3);
    Eigen::MatrixXd minV2;
    minV2.setZero(minRows,3);
    int Pcounter = 0;
    for(int i=0; i<minRows;i++){
        Pcounter = i * floor(V2.rows()/minRows);
        minV1(i) = V1(i);
        minV2(i) = V2(Pcounter);
        A(i,0) = n(i,2)*V1(i,1)-n(i,1)*V1(i,2);
        A(i,1) = n(i,0)*V1(i,2)-n(i,2)*V1(i,0);
        A(i,2) = n(i,1)*V1(i,0)-n(i,0)*V1(i,1);
        A(i,3) = n(i,0);
        A(i,4) = n(i,1);
        A(i,5) = n(i,2);
        b(i,0)= n(i,0)*V2(Pcounter,0)+n(i,1)*V2(Pcounter,1)+n(i,2)*V2(Pcounter,2) -n(i,0)*V1(i,0) -n(i,1)*V1(i,1) -n(i,2)*V1(i,2);
    }

    float v1XMean = minV1.col(0).mean();
    float v1YMean = minV1.col(1).mean();
    float v1ZMean = minV1.col(2).mean();
    Eigen::Vector3d v1Mean;
    v1Mean << v1XMean, v1YMean, v1ZMean;
    
    //V2 = T*V1;
    float v2XMean =minV2.col(0).mean();
    float v2YMean =minV2.col(1).mean();
    float v2ZMean =minV2.col(2).mean();
    Eigen::Vector3d v2Mean;
    v2Mean << v2XMean, v2YMean, v2ZMean;
//    SVDC = ((minV1-v1Mean.replicate(1,minRows).transpose()).transpose())*(minV2-v2Mean.replicate(1,minRows).transpose());
//    cout << "The SVDC is"<<endl << SVDC << endl;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd SVDU = svd.matrixU();
    Eigen::MatrixXd SVDV = svd.matrixV();
    Eigen::MatrixXd SVDS = svd.singularValues().asDiagonal().inverse();
//    SVDS = SVDS.diagonal().inverse();
    AP = SVDV*SVDS*SVDU.transpose();
    x = AP*b;
    x = -x;
   //    x=(A.transpose()*A).inverse()*A.transpose()*b;
    Eigen::MatrixXd T;
    T.setZero(3,1);
    T(0)=x(3);
    T(1)=x(4);
    T(2)=x(5);
//    Eigen::MatrixXd rx;
//    rx.setZero(3,3);
//    Eigen::MatrixXd ry;
//    ry.setZero(3,3);
//    Eigen::MatrixXd rz;
//    rz.setZero(3,3);
    Eigen::Matrix3d R;

    cout << "x"<<endl << x <<endl;

    double sina = sin(x(0));
    double cosa = cos(x(0));
    double sinb=sin(x(1));
    double cosb = cos(x(1));
    double sinc=sin(x(2));
    double cosc = cos(x(2));
    R(0,0) =cosc*cosb;
    R(0,1) = -sinc*cosa+cosc*sinb*sina;
    R(0,2) = sinc*sina+cosc*sinb*cosa;
    R(1,0) = sinc*cosb;
    R(1,1) = cosc*cosa+sinc*sinb*sina;
    R(1,2) = -cosc*sina+sinc*sinb*cosa;
    R(2,0) = -sinb;
    R(2,1) = cosb*sina;
    R(2,2) = cosb*cosa;
    

    cout<< "T: "<<endl<<T<<endl;
//    R << r11,r12,r13,r21,r22,r23,r31,r32,r33;
    cout<<"R:"<<endl<<R<<endl;

    V1 = (V1 - T.replicate(1,V1.rows()).transpose())*R;
    return V1;
}
acq::NeighboursT normalAnn::inserNeighbours(Eigen::MatrixXd V1,int kneighbours){
    acq::NeighboursT neighbours;
    int nPts;
    ANNpointArray	targetArray;		// target data points array
    ANNpoint		queryPt;				// query point
    ANNidxArray		neighbourInd;					// near neighbor indices
    ANNdistArray	neighbourDis;					// near neighbor distances
    ANNkd_tree*		kdTree;					// search tree
    ANNpoint        pt;
    
    //Initial variables
    pt = annAllocPt(3);
    queryPt = annAllocPt(3);
    neighbourInd = new ANNidx[kneighbours];
    neighbourDis = new ANNdist[kneighbours];
    
    nPts = 0;
    targetArray = annAllocPts(V1.rows(), 3);
    
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
    for(int i=0;i <V1.rows();i++){
        acq::NeighboursT::mapped_type currNeighbours;
        queryPt = targetArray[i];
        //            cout<<"Source: " << i<<endl;
        kdTree->annkSearch(queryPt, kneighbours+1, neighbourInd, neighbourDis);
        //            cout<<"target: "<<neighbourInd[0]<<" No.2: "<<neighbourInd[1]<<endl<<endl;
        for (int j=0; j<kneighbours+1; j++) {
            if (neighbourInd[j] == i) {
                continue;
            }
            currNeighbours.insert(neighbourInd[j]);
        }
        neighbours.insert(std::make_pair(i,currNeighbours));
    }
    
    return neighbours;
    
}
