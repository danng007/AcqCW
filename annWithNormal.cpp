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
//    Eigen::MatrixXd minV1;
//    minV1.setZero(minRows,3);
//    Eigen::MatrixXd minV2;
//    minV2.setZero(minRows,3);
    
    //Ann search, find V1 V2 neighbours
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
    neighbourInd = new ANNidx[1];
    neighbourDis = new ANNdist[1];

    sourceArray = annAllocPts(minRows, 3);
    targetArray = annAllocPts(V2.rows(), 3);
    for (int i=0; i<minRows; i++) {
        pt = sourceArray[i];
        for(int d=0; d<3;d++){
            pt[d] = V1(round(i*(V1.rows()/minRows)),d); //Sample the number of points
            //            pt[d] = V2(i,d);
        }
        sourceArray[i]=pt;
    }
    
    for (int i=0; i<V2.rows(); i++) {
        pt = targetArray[i];
        for(int d=0; d<3;d++){
            pt[d] = V2(i,d);
        }
        targetArray[i]=pt;
    }
    
    kdTree = new ANNkd_tree(
                            targetArray,
                            V2.rows(),
                            3
                            );
    Eigen::MatrixXd V3; //neighbour Vs
    V3.setZero(minRows,3);
    Eigen::MatrixXd newV2; //neighbour Vs
    newV2.setZero(minRows,3);
    //i->V1-<queryPt index  neighbourInd[0]->V2->pt index
    for(int i=0;i <minRows;i++){
        
        queryPt = sourceArray[i];
        //            cout<<"Source: " << i<<endl;
        kdTree->annkSearch(queryPt, 1, neighbourInd, neighbourDis);
        //            cout<<"target: "<<neighbourInd[0]<<" No.2: "<<neighbourInd[1]<<endl<<endl;
        pt = targetArray[neighbourInd[0]];
        
        A(i,0) = n(neighbourInd[0],2)*V1(i,1)-n(neighbourInd[0],1)*V1(i,2);
        A(i,1) = n(neighbourInd[0],0)*V1(i,2)-n(neighbourInd[0],2)*V1(i,0);
        A(i,2) = n(neighbourInd[0],1)*V1(i,0)-n(neighbourInd[0],0)*V1(i,1);
        A(i,3) = n(neighbourInd[0],0);
        A(i,4) = n(neighbourInd[0],1);
        A(i,5) = n(neighbourInd[0],2);
        b(i,0)= n(neighbourInd[0],0)*V2(neighbourInd[0],0)+n(neighbourInd[0],1)*V2(neighbourInd[0],1)+n(neighbourInd[0],2)*V2(neighbourInd[0],2) -n(neighbourInd[0],0)*V1(i,0) -n(neighbourInd[0],1)*V1(i,1) -n(neighbourInd[0],2)*V1(i,2);
    }
        //Ann search, find V1 V2 neighbours

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
