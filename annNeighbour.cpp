#include "annNeighbour.h"
using namespace std;
using namespace acq;
//
void getsampledAnnArray(CloudT const& cloud,ANNpointArray &dataArray,int rows)
{
    ANNpoint Pt;
    Pt = annAllocPt(3);
    
    
    //downsampling count
    int count = 0;
    
    for (int i =0;i<rows;i++)
    {
       
            //Pt get the space of data array
            Pt = dataArray[count];
            //Pt get the coordinates of mesh point
        
                Pt[0] =  cloud.row(i).x();
                Pt[1] =  cloud.row(i).y();
                Pt[2] =  cloud.row(i).z();
            //assign Pt coordinates to data array
            dataArray[count] = Pt;
    }
};

void calculateCloudNeighbours(
                         CloudT  const& cloud,
                         CloudT  const& cloud2,
                                 int k,
                                 int rows)
{
    int nPts;
    ANNpointArray	sourceArray;		// source data points array
    ANNpointArray	targetArray;		// target data points array
    ANNpointArray	MatchArray;				// matched data points array
    ANNpoint		queryPt;				// query point
    ANNpoint		matchPt;				// match point
    ANNidxArray		neighbourInd;					// near neighbor indices
    ANNdistArray	neighbourDis;					// near neighbor distances
    ANNkd_tree*		kdTree;					// search structure
    
    //Initial variables
    queryPt = annAllocPt(3);
    matchPt = annAllocPt(3);
    neighbourInd = new ANNidx[k];					// allocate near neigh indices
    neighbourDis = new ANNdist[k];
    
    nPts = 0;									// read data points
    sourceArray = annAllocPts(rows, 3);
    targetArray = annAllocPts(rows, 3);
    MatchArray  = annAllocPts(rows, 3);
    
    //assign sampled meshes to ANN array,directly modify the address of arrays
//    
    getsampledAnnArray(cloud,sourceArray,rows);
    getsampledAnnArray(cloud2,targetArray,rows);
    

    
}