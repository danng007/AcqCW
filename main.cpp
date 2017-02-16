#include "acq/normalEstimation.h"
#include "acq/decoratedCloud.h"
#include "acq/cloudManager.h"

#include "nanogui/formhelper.h"
#include "nanogui/screen.h"

#include "igl/readOFF.h"
#include "igl/viewer/Viewer.h"
#include <ANN/ANN.h>
#include <iostream>
#include "annFindNeighbour.hpp"
#include "annWithNormal.hpp"
#include <random>
#include <iterator>
#include "multipleAlign.hpp"
//#include "annNeighbour.h"
using namespace std;
namespace acq {

/** \brief                      Re-estimate normals of cloud \p V fitting planes
 *                              to the \p kNeighbours nearest neighbours of each point.
 * \param[in ] kNeighbours      How many neighbours to use (Typiclaly: 5..15)
 * \param[in ] vertices         Input pointcloud. Nx3, where N is the number of points.
 * \param[in ] maxNeighbourDist Maximum distance between vertex and neighbour.
 * \param[out] viewer           The viewer to show the normals at.
 * \return                      The estimated normals, Nx3.
 */
NormalsT
recalcNormals(
    int                 const  kNeighbours,
    CloudT              const& vertices,
    float               const  maxNeighbourDist
) {
    NeighboursT const neighbours =
        calculateCloudNeighbours(
            /* [in]        cloud: */ vertices,
            /* [in] k-neighbours: */ kNeighbours,
            /* [in]      maxDist: */ maxNeighbourDist
        );

    // Estimate normals for points in cloud vertices
    NormalsT normals =
        calculateCloudNormals(
            /* [in]               Cloud: */ vertices,
            /* [in] Lists of neighbours: */ neighbours
        );

    return normals;
} //...recalcNormals()

void setViewerNormals(
    igl::viewer::Viewer      & viewer,
    CloudT              const& vertices,
    NormalsT            const& normals
) {
    // [Optional] Set viewer face normals for shading
    //viewer.data.set_normals(normals);

    // Clear visualized lines (see Viewer.clear())
    viewer.data.lines = Eigen::MatrixXd(0, 9);

    // Add normals to viewer
    viewer.data.add_edges(
        /* [in] Edge starting points: */ vertices,
        /* [in]       Edge endpoints: */ vertices + normals * 0.01, // scale normals to 1% length
        /* [in]               Colors: */ Eigen::Vector3d::Zero()
    );
}

} //...ns acq

int main(int argc, char *argv[]) {
  
    // How many neighbours to use for normal estimation, shown on GUI.
    acq::NormalsT cloud1N;
    int kNeighbours = 10;
    // Maximum distance between vertices to be considered neighbours (FLANN mode)
    float maxNeighbourDist = 0.15; //TODO: set to average vertex distance upon read

    // Dummy enum to demo GUI
    enum Orientation { Up=0, Down, Left, Right } dir = Up;
    // Dummy variable to demo GUI
    bool boolVariable = true;
    // Dummy variable to demo GUI
    double disMean = 0.0;
    double disDiv = 0.0;
    float floatVariable = 0.1f;
    double transX =0.0;
    double transY = 0.0;
    double transZ =0.0;
	Eigen::Vector3d T;
	T << 0.1f, 0.0f, 0.02f;
    double sampleRatio = 1.0;
    double rotateZ = 0.0;
////    Eigen::Matrix3d R;
//    R << cos(M_PI/180*rotateY), 0, sin(M_PI/180*rotateY),
//    0, 1, 0,
//    -(sin(M_PI/180*rotateY)), 0, cos(M_PI/180*rotateY);
    // Load a mesh in OFF format
    std::string meshPath = "../3rdparty/libigl/tutorial/shared/bun000.off";
	std::string newMesh = "../3rdparty/libigl/tutorial/shared/bun045.off";
    string mesh3 = "../3rdparty/libigl/tutorial/shared/bun090.off";
    string mesh4 = "../3rdparty/libigl/tutorial/shared/bun270.off";
    string mesh5 = "../3rdparty/libigl/tutorial/shared/bun315.off";

    if (argc > 1) {
        meshPath = std::string(argv[1]);
        if (meshPath.find(".off") == std::string::npos) {
            std::cerr << "Only ready for  OFF files for now...\n";
            return EXIT_FAILURE;
        }
    } else {
        std::cout << "Usage: iglFrameWork <path-to-off-mesh.off>." << "\n";
    }
  

    // Visualize the mesh in a viewer
    igl::viewer::Viewer viewer;
    {
        // Don't show face edges
        viewer.core.show_lines = false;
    }

    // Store cloud so we can store normals later
    acq::CloudManager cloudManager;
    // Read mesh from meshPath
    {
        // Pointcloud vertices, N rows x 3 columns.
        Eigen::MatrixXd V1;
		Eigen::MatrixXd V2;
        Eigen::MatrixXd V3;
        Eigen::MatrixXd V4;
        Eigen::MatrixXd V5;
        // Face indices, M x 3 integers referring to V.
        Eigen::MatrixXi F1;
		Eigen::MatrixXi F2;
        Eigen::MatrixXi F3;
        Eigen::MatrixXi F4;
        Eigen::MatrixXi F5;
        // Read mesh
        igl::readOFF(meshPath, V1, F1);
        igl::readOFF(mesh3, V3, F3);
        igl::readOFF(mesh4, V4, F4);
        igl::readOFF(mesh5, V5, F5);
        
		if (V1.rows() <= 0) {
			std::cerr << "Could not read mesh at " << meshPath
				<< "...exiting...\n";
			return EXIT_FAILURE;
		} //...if vertices read
        igl::readOFF(newMesh, V2, F2);
        if (V1.rows() <= 0) {
            std::cerr << "Could not read mesh at " << newMesh
            << "...exiting...\n";
            return EXIT_FAILURE;
        } //...if vertices read
//        V2 = (V2+T.replicate(1,V2.rows()).transpose());
        T << 0.0f, 0.1f, 0.02f;
        V3 = (V3+T.replicate(1,V3.rows()).transpose());
        T << -0.1f, 0.1f, -0.02f;
        V4 = (V4+T.replicate(1,V4.rows()).transpose());
        T << -0.12f, -0.1f, 0.03f;
        V5 = (V5+T.replicate(1,V5.rows()).transpose());
        cloudManager.addCloud(acq::DecoratedCloud(V1,F1));
       
        cloudManager.getCloud(0).setNormals(
                                            acq::recalcNormals(
                                                               /* [in]      k-neighbours for flann: */ 10,
                                                               /* [in]             vertices matrix: */ V1,
                                                               /* [in]      max neighbour distance: */ maxNeighbourDist
                                                               )
                                            );
//        acq::NeighboursT neighbours;
//        neighbours=normalAnn::inserNeighbours(V1,10);
//        cloud1N = acq::calculateCloudNormals(V1,neighbours);
        
        cloud1N =  cloudManager.getCloud(0).getNormals();
        
        cloudManager.addCloud(acq::DecoratedCloud(V2,F2));
        cloudManager.addCloud(acq::DecoratedCloud(V1,F1));
        cloudManager.addCloud(acq::DecoratedCloud(V2,F2));
        cloudManager.setCloud(acq::DecoratedCloud(V3,F3),4);
        cloudManager.setCloud(acq::DecoratedCloud(V4,F4),5);
        cloudManager.setCloud(acq::DecoratedCloud(V5,F5),6);
        
        //----------------Point Processing-----------//


		Eigen::MatrixXd totalV(V1.rows() + V2.rows(), V1.cols());
		totalV << V1, V2;
		Eigen::MatrixXi totalF(F1.rows() + F2.rows(), F1.cols());
        totalF << F1, (F2.array()+V1.rows());
		//set different colors to two objects
		Eigen::MatrixXd C(totalF.rows(), 3);
		C <<
        Eigen::RowVector3d(0.0, 0.3, 1.0).replicate(F1.rows(), 1),
        Eigen::RowVector3d(1.0, 1.0, 0.0).replicate(F2.rows(), 1);
        // Store read vertices and faces
        cloudManager.addCloud(acq::DecoratedCloud(totalV, totalF));
		
        // Show mesh
		viewer.data.set_mesh(
			totalV,
            totalF
        );
        
		viewer.data.set_colors(C);
//		 Calculate normals on launch
//		cloudManager.getCloud(0).setNormals(
//			acq::recalcNormals(
//				/* [in]      K-neighbours for FLANN: */ kNeighbours,
//				/* [in]             Vertices matrix: */ cloudManager.getCloud(0).getVertices(),
//				/* [in]      max neighbour distance: */ maxNeighbourDist
//			)
//		);

        
//Process Normals ICP
        
		// Update viewer
//		acq::setViewerNormals(
//			viewer,
//			cloudManager.getCloud(0).getVertices(),
//			cloudManager.getCloud(0).getNormals()
//		);

        
    } //...read mesh

	

    // Extend viewer menu using a lambda function
    viewer.callback_init =
        [
            &cloudManager, &kNeighbours, &maxNeighbourDist,
         &floatVariable, &boolVariable, &dir, &disDiv,&disMean,&cloud1N, &rotateZ,&transX,&transY,&transZ, &sampleRatio
        ] (igl::viewer::Viewer& viewer)
    {
        // Add an additional menu window
        viewer.ngui->addWindow(Eigen::Vector2i(900,10), "Acquisition3D");
        viewer.ngui->addGroup("Initial Cloud");
        
        viewer.ngui->addVariable<double>(
                                      /* Displayed name: */ "Rotation Z",
                                      
                                      /*  Setter lambda: */ [&] (double val) {

                                          rotateZ= val;
                                      },
                                      /*  Getter lambda: */ [&]() {
                                          return rotateZ; // get
                                      } );
        viewer.ngui->addVariable<double>(
                                         /* Displayed name: */ "Translation X",
                                         
                                         /*  Setter lambda: */ [&] (double val) {
                                             
                                             transX= val;
                                         },
                                         /*  Getter lambda: */ [&]() {
                                             return transX; // get
                                         } );
        viewer.ngui->addVariable<double>(
                                         /* Displayed name: */ "Translation Y",
                                         
                                         /*  Setter lambda: */ [&] (double val) {
                                             
                                             transY= val;
                                         },
                                         /*  Getter lambda: */ [&]() {
                                             return transY; // get
                                         } );
        viewer.ngui->addVariable<double>(
                                         /* Displayed name: */ "Translation Z",
                                         
                                         /*  Setter lambda: */ [&] (double val) {
                                             
                                             transZ= val;
                                         },
                                         /*  Getter lambda: */ [&]() {
                                             return transZ; // get
                                         } );

        viewer.ngui->addButton("Start Initial",
                               [&]() {
                                   Eigen::Matrix3d Rotation;
                                   Rotation<<Eigen::AngleAxisd(rotateZ*M_PI/180,Eigen::Vector3d(0,0,1)).toRotationMatrix();
                                   Eigen::Vector3d T;
                                   T << transX,transY,transZ;
                                   acq::DecoratedCloud &cloud1 = cloudManager.getCloud(2);
                                   acq::DecoratedCloud &cloud2 = cloudManager.getCloud(3);
                                   Eigen::MatrixXi F1 = cloud1.getFaces();
                                   Eigen::MatrixXi F2 = cloud2.getFaces();
                                   Eigen:: MatrixXd V1;
                                   V1 = cloud1.getVertices();
                                   Eigen:: MatrixXd V2;
                                   V2 = cloud2.getVertices();
                                   V2 = (V2+T.replicate(1,V2.rows()).transpose())*Rotation;
                                   
                                   cloudManager.setCloud(acq::DecoratedCloud(V1,F1),0);
                                   cloudManager.setCloud(acq::DecoratedCloud(V2,F2),1);
                                   
                                   Eigen::MatrixXd totalV(V1.rows() + V2.rows(), V1.cols());
                                   totalV << V1, V2;
                                   Eigen::MatrixXi totalF(F1.rows() + F2.rows(), F1.cols());
                                   totalF << F1, (F2.array()+V1.rows());
                                   //set different colors to two objects
                                   Eigen::MatrixXd C(totalF.rows(), 3);
                                   C <<
                                   Eigen::RowVector3d(0.0, 0.3, 1.0).replicate(F1.rows(), 1),
                                   Eigen::RowVector3d(1.0, 1.0, 0.0).replicate(F2.rows(), 1);
                                   
                                   // Store read vertices and faces
                                   cloudManager.addCloud(acq::DecoratedCloud(totalV, totalF));
                                   
                                   // Show mesh
                                   viewer.data.clear();
                                   viewer.data.set_mesh(
                                                        totalV,
                                                        totalF
                                                        );
                                   
                                   viewer.data.set_colors(C);
                                   
                                   
                               }
         );

        viewer.ngui->addGroup("New Bee Coursework");
        viewer.ngui->addVariable<double>(
                                         /* Displayed name: */ "Sample ratio",
                                         
                                         /*  Setter lambda: */ [&] (double val) {
                                             
                                             sampleRatio= val;
                                         },
                                         /*  Getter lambda: */ [&]() {
                                             return sampleRatio; // get
                                         } );
        viewer.ngui->addVariable<double>(
                                         /* Displayed name: */ "Distribution Mean",
                                         
                                         /*  Setter lambda: */ [&] (double val) {
                                             
                                             disMean= val;
                                         },
                                         /*  Getter lambda: */ [&]() {
                                             return disMean; // get
                                         } );
        viewer.ngui->addVariable<double>(
                                         /* Displayed name: */ "Distribution Variation",
                                         
                                         /*  Setter lambda: */ [&] (double val) {
                                             
                                             disDiv= val;
                                         },
                                         /*  Getter lambda: */ [&]() {
                                             return disDiv; // get
                                         } );

        viewer.ngui->addButton("Add Noisy",
                               [&]() {
                                   acq::DecoratedCloud &cloud1 = cloudManager.getCloud(0);
                                   acq::DecoratedCloud &cloud2 = cloudManager.getCloud(1);
                                   Eigen::MatrixXi F1 = cloud1.getFaces();
                                   Eigen::MatrixXi F2 = cloud2.getFaces();
                                   Eigen:: MatrixXd V1;
                                   V1 = cloud1.getVertices();
                                   Eigen:: MatrixXd V2;
                                   V2 = cloud2.getVertices();
                                  
                                  
                                   default_random_engine randomGen;
                                   normal_distribution<double> dis(disMean,disDiv);
                                   Eigen::Vector3d noisy;
                                   for(int i =0;i<V2.rows();i++){
                                       noisy[0] = dis(randomGen)/1000;
                                       noisy[1] = dis(randomGen)/1000;
                                       noisy[2] = dis(randomGen)/1000;
                                       V2.row(i) += noisy;
                                       
                                   }
                                   
                                   cloudManager.setCloud(acq::DecoratedCloud(V1,F1),0);
                                   cloudManager.setCloud(acq::DecoratedCloud(V2,F2),1);
                                   
                                   Eigen::MatrixXd totalV(V1.rows() + V2.rows(), V1.cols());
                                   totalV << V1, V2;
                                   Eigen::MatrixXi totalF(F1.rows() + F2.rows(), F1.cols());
                                   totalF << F1, (F2.array()+V1.rows());
                                   //set different colors to two objects
                                   Eigen::MatrixXd C(totalF.rows(), 3);
                                   C <<
                                   Eigen::RowVector3d(0.0, 0.3, 1.0).replicate(F1.rows(), 1),
                                   Eigen::RowVector3d(1.0, 1.0, 0.0).replicate(F2.rows(), 1);
                                   
                                   // Store read vertices and faces
                                   cloudManager.addCloud(acq::DecoratedCloud(totalV, totalF));
                                   
                                   // Show mesh
                                   viewer.data.clear();
                                   viewer.data.set_mesh(
                                                        totalV,
                                                        totalF
                                                        );
                                   
                                   viewer.data.set_colors(C);
                                   
                                   
                               }
                               );

         viewer.ngui->addButton("Point to Point",
                               [&]() {
                                   acq::DecoratedCloud &cloud1 = cloudManager.getCloud(0);
                                   acq::DecoratedCloud &cloud2 = cloudManager.getCloud(1);
                                   Eigen::MatrixXi F1 = cloud1.getFaces();
                                   Eigen::MatrixXi F2 = cloud2.getFaces();
                                   Eigen:: MatrixXd V1;
                                   V1 = cloud1.getVertices();
                                   Eigen:: MatrixXd V2;
                                   V2 = cloud2.getVertices();
                                   double timeBegin = std::clock();
                                   
                                   //ANN TEST
                                   int minRows = min(V1.rows(),V2.rows())*sampleRatio;
                                   Eigen::MatrixXd V3;
                                   Eigen::MatrixXd newV2;
                                   int annKNeighbours = 1;
                                   std::tie(V3,newV2)= ann::annNeighbour(V1,V2,annKNeighbours,minRows);
                                   //ANN TEST
                                   V2 = ann::annSVD(V3,newV2,V2);
                                   
                                   double timeLast = (std::clock() - timeBegin)*1.0/CLOCKS_PER_SEC;
                                   std::cout << "*************************************" << endl;
                                   std::cout << "Processing Time: " << timeLast << " s" << endl;
                                   std::cout << "*************************************" << endl;
                                   
                                   cloudManager.setCloud(acq::DecoratedCloud(V1,F1),0);
                                   cloudManager.setCloud(acq::DecoratedCloud(V2,F2),1);
                                   
                                   Eigen::MatrixXd totalV(V1.rows() + V2.rows(), V1.cols());
                                   totalV << V1, V2;
                                   Eigen::MatrixXi totalF(F1.rows() + F2.rows(), F1.cols());
                                   totalF << F1, (F2.array()+V1.rows());
                                   //set different colors to two objects
                                   Eigen::MatrixXd C(totalF.rows(), 3);
                                   C <<
                                   Eigen::RowVector3d(0.0, 0.3, 1.0).replicate(F1.rows(), 1),
                                   Eigen::RowVector3d(1.0, 1.0, 0.0).replicate(F2.rows(), 1);
                                   
                                   // Store read vertices and faces
                                   cloudManager.addCloud(acq::DecoratedCloud(totalV, totalF));
                                   
                                   // Show mesh
                                   viewer.data.clear();
                                   viewer.data.set_mesh(
                                                        totalV,
                                                        totalF
                                                        );
                                   
                                   viewer.data.set_colors(C);
                                   
                                   
                               }
                               );
        viewer.ngui->addButton("Point to Plane (Normals)",
                               [&]() {
                                   acq::DecoratedCloud &cloud1 = cloudManager.getCloud(0);
                                   acq::DecoratedCloud &cloud2 = cloudManager.getCloud(1);
                                   Eigen::MatrixXi F1 = cloud1.getFaces();
                                   Eigen::MatrixXi F2 = cloud2.getFaces();
                                   Eigen:: MatrixXd V1;
                                   V1 = cloud1.getVertices();
                                   Eigen:: MatrixXd V2;
                                   V2 = cloud2.getVertices();
                                   double timeBegin = std::clock();
                                   
                                       int minRows = min(V1.rows(),V2.rows())*sampleRatio;
                                       
                                       //                                   V2 = normalAnn::calRT(V1,V2,normals,minRows);
                                       V2 = normalAnn::calRT(V1,V2,cloud1N,minRows);
    
                                   
                                   
                                   double timeLast = (std::clock() - timeBegin)*1.0/CLOCKS_PER_SEC;
                                   std::cout << "*************************************" << endl;
                                   std::cout << "Processing Time: " << timeLast << " s" << endl;
                                   std::cout << "*************************************" << endl;
                                   
                                   cloudManager.setCloud(acq::DecoratedCloud(V1,F1),0);
                                   cloudManager.setCloud(acq::DecoratedCloud(V2,F2),1);
                                   
                                   Eigen::MatrixXd totalV(V1.rows() + V2.rows(), V1.cols());
                                   totalV << V1, V2;
                                   Eigen::MatrixXi totalF(F1.rows() + F2.rows(), F1.cols());
                                   totalF << F1, (F2.array()+V1.rows());
                                   //set different colors to two objects
                                   Eigen::MatrixXd C(totalF.rows(), 3);
                                   C <<
                                   Eigen::RowVector3d(0.0, 0.3, 1.0).replicate(F1.rows(), 1),
                                   Eigen::RowVector3d(1.0, 1.0, 0.0).replicate(F2.rows(), 1);
                                   
                                   // Store read vertices and faces
                                   cloudManager.addCloud(acq::DecoratedCloud(totalV, totalF));
                                   
                                   // Show mesh
                                   viewer.data.clear();
                                   viewer.data.set_mesh(
                                                        totalV,
                                                        totalF
                                                        );
                                   
                                   viewer.data.set_colors(C);
                                   
                                   
                               }
                               );
        viewer.ngui->addGroup("Multiple Align");
        viewer.ngui->addButton("Multiple Align Initial",
                               [&]() {
                                   acq::DecoratedCloud &cloud1 = cloudManager.getCloud(0);
                                   acq::DecoratedCloud &cloud2 = cloudManager.getCloud(1);
                                   acq::DecoratedCloud &cloud3 = cloudManager.getCloud(4);
                                   acq::DecoratedCloud &cloud4 = cloudManager.getCloud(5);
                                   acq::DecoratedCloud &cloud5 = cloudManager.getCloud(6);
                                   
                                   Eigen::MatrixXi F1 = cloud1.getFaces();
                                   Eigen::MatrixXi F2 = cloud2.getFaces();
                                   Eigen::MatrixXi F3 = cloud3.getFaces();
                                   Eigen::MatrixXi F4 = cloud4.getFaces();
                                   Eigen::MatrixXi F5 = cloud5.getFaces();
                                   
                                   Eigen:: MatrixXd V1;
                                   V1 = cloud1.getVertices();
                                   Eigen:: MatrixXd V2;
                                   V2 = cloud2.getVertices();
                                   Eigen:: MatrixXd V3;
                                   V3 = cloud3.getVertices();
                                   Eigen:: MatrixXd V4;
                                   V4 = cloud4.getVertices();
                                   Eigen:: MatrixXd V5;
                                   V5 = cloud5.getVertices();
                                   
                                
                                   
                                   cloudManager.setCloud(acq::DecoratedCloud(V1,F1),0);
                                   cloudManager.setCloud(acq::DecoratedCloud(V2,F2),1);
                                   cloudManager.setCloud(acq::DecoratedCloud(V3,F3),4);
                                   cloudManager.setCloud(acq::DecoratedCloud(V4,F4),5);
                                   cloudManager.setCloud(acq::DecoratedCloud(V5,F5),6);
                                   
                                   Eigen::MatrixXd totalV(V1.rows() + V2.rows() + V3.rows()+V4.rows()+V5.rows(), V1.cols());
                                   totalV << V1, V2,V3,V4,V5;
                                   Eigen::MatrixXi totalF(F1.rows() + F2.rows()+F3.rows()+F4.rows()+F5.rows(), F1.cols());
                                   totalF << F1, (F2.array()+V1.rows()), (F3.array()+V2.rows()+V1.rows()), (F4.array()+V3.rows()+V2.rows()+V1.rows()), (F5.array()+V4.rows()+V3.rows()+V2.rows()+V1.rows());
                                   //set different colors to two objects
                                   Eigen::MatrixXd C(totalF.rows(), 3);
                                   C <<
                                   Eigen::RowVector3d(0.0, 0.3, 1.0).replicate(F1.rows(), 1),
                                   Eigen::RowVector3d(1.0, 1.0, 0.0).replicate(F2.rows(), 1),
                                   Eigen::RowVector3d(0.3, 1.0, 0.0).replicate(F3.rows(), 1),
                                   Eigen::RowVector3d(1.0, 0.0, 0.2).replicate(F4.rows(), 1),
                                   Eigen::RowVector3d(0.1, 0.4, 0.6).replicate(F5.rows(), 1);
                                   
                                   // Store read vertices and faces
                                   //                                   cloudManager.addCloud(acq::DecoratedCloud(totalV, totalF));
                                   
                                   // Show mesh
                                   viewer.data.clear();
                                   viewer.data.set_mesh(
                                                        totalV,
                                                        totalF
                                                        );
                                   
                                   viewer.data.set_colors(C);
                                   
                               }
                               );

        viewer.ngui->addButton("Multiple Align Step 1",
                               [&]() {
                                   acq::DecoratedCloud &cloud1 = cloudManager.getCloud(0);
                                   acq::DecoratedCloud &cloud2 = cloudManager.getCloud(1);
                                   acq::DecoratedCloud &cloud3 = cloudManager.getCloud(4);
                                   acq::DecoratedCloud &cloud4 = cloudManager.getCloud(5);
                                   acq::DecoratedCloud &cloud5 = cloudManager.getCloud(6);

                                   Eigen::MatrixXi F1 = cloud1.getFaces();
                                   Eigen::MatrixXi F2 = cloud2.getFaces();
                                   Eigen::MatrixXi F3 = cloud3.getFaces();
                                   Eigen::MatrixXi F4 = cloud4.getFaces();
                                   Eigen::MatrixXi F5 = cloud5.getFaces();
                                   
                                   Eigen:: MatrixXd V1;
                                   V1 = cloud1.getVertices();
                                   Eigen:: MatrixXd V2;
                                   V2 = cloud2.getVertices();
                                   Eigen:: MatrixXd V3;
                                   V3 = cloud3.getVertices();
                                   Eigen:: MatrixXd V4;
                                   V4 = cloud4.getVertices();
                                   Eigen:: MatrixXd V5;
                                   V5 = cloud5.getVertices();
                                   
                                   V1 = mul::deMean(V1);
                                   V2 = mul::deMean(V2);
                                   V3 = mul::deMean(V3);
                                   V4 = mul::deMean(V4);
                                   V5 = mul::deMean(V5);

                                   cloudManager.setCloud(acq::DecoratedCloud(V1,F1),0);
                                   cloudManager.setCloud(acq::DecoratedCloud(V2,F2),1);
                                   cloudManager.setCloud(acq::DecoratedCloud(V3,F3),4);
                                   cloudManager.setCloud(acq::DecoratedCloud(V4,F4),5);
                                   cloudManager.setCloud(acq::DecoratedCloud(V5,F5),6);
                                   
                                   Eigen::MatrixXd totalV(V1.rows() + V2.rows() + V3.rows()+V4.rows()+V5.rows(), V1.cols());
                                   totalV << V1, V2,V3,V4,V5;
                                   Eigen::MatrixXi totalF(F1.rows() + F2.rows()+F3.rows()+F4.rows()+F5.rows(), F1.cols());
                                   totalF << F1, (F2.array()+V1.rows()), (F3.array()+V2.rows()+V1.rows()), (F4.array()+V3.rows()+V2.rows()+V1.rows()), (F5.array()+V4.rows()+V3.rows()+V2.rows()+V1.rows());
                                   //set different colors to two objects
                                   Eigen::MatrixXd C(totalF.rows(), 3);
                                   C <<
                                   Eigen::RowVector3d(0.0, 0.3, 1.0).replicate(F1.rows(), 1),
                                   Eigen::RowVector3d(1.0, 1.0, 0.0).replicate(F2.rows(), 1),
                                   Eigen::RowVector3d(0.3, 1.0, 0.0).replicate(F3.rows(), 1),
                                   Eigen::RowVector3d(1.0, 0.0, 0.2).replicate(F4.rows(), 1),
                                   Eigen::RowVector3d(0.1, 0.4, 0.6).replicate(F5.rows(), 1);
                                   
                                   // Store read vertices and faces
//                                   cloudManager.addCloud(acq::DecoratedCloud(totalV, totalF));
                                   
                                   // Show mesh
                                   viewer.data.clear();
                                   viewer.data.set_mesh(
                                                        totalV,
                                                        totalF
                                                        );
                                   
                                   viewer.data.set_colors(C);
                                   
                               }
                            );
        viewer.ngui->addButton("Multiple Align Step 2",
                               [&]() {
                                   acq::DecoratedCloud &cloud1 = cloudManager.getCloud(0);
                                   acq::DecoratedCloud &cloud2 = cloudManager.getCloud(1);
                                   acq::DecoratedCloud &cloud3 = cloudManager.getCloud(4);
                                   acq::DecoratedCloud &cloud4 = cloudManager.getCloud(5);
                                   acq::DecoratedCloud &cloud5 = cloudManager.getCloud(6);
                                   
                                   Eigen::MatrixXi F1 = cloud1.getFaces();
                                   Eigen::MatrixXi F2 = cloud2.getFaces();
                                   Eigen::MatrixXi F3 = cloud3.getFaces();
                                   Eigen::MatrixXi F4 = cloud4.getFaces();
                                   Eigen::MatrixXi F5 = cloud5.getFaces();
                                   
                                   Eigen:: MatrixXd V1;
                                   V1 = cloud1.getVertices();
                                   Eigen:: MatrixXd V2;
                                   V2 = cloud2.getVertices();
                                   Eigen:: MatrixXd V3;
                                   V3 = cloud3.getVertices();
                                   Eigen:: MatrixXd V4;
                                   V4 = cloud4.getVertices();
                                   Eigen:: MatrixXd V5;
                                   V5 = cloud5.getVertices();
                                   
                                   Eigen::MatrixXd VV;
                                   Eigen::MatrixXd annV1;
                                   Eigen::MatrixXd annV2;
                                   int minRows;
                                   for (int i=0;i<5;i++){
                                       minRows = min(V4.rows(),V5.rows())*sampleRatio;
                                       tie(annV1,annV2)= ann::annNeighbour(V5,V4,1,minRows);
                                       //ANN TEST
                                       V4 = ann::annSVD(annV1,annV2,V4);
                                       
                                       minRows = min(V2.rows(),V3.rows())*sampleRatio;
                                       tie(annV1,annV2)= ann::annNeighbour(V2,V3,1,minRows);
                                       //ANN TEST
                                       V3 = ann::annSVD(annV1,annV2,V3);
                                       
                                       minRows = min(V1.rows(),V5.rows())*sampleRatio;
                                       tie(annV1,annV2)= ann::annNeighbour(V1,V5,1,minRows);
                                       //ANN TEST
                                       V5 = ann::annSVD(annV1,annV2,V5);
                                       
                                       minRows = min(V1.rows(),V2.rows())*sampleRatio;
                                       tie(annV1,annV2)= ann::annNeighbour(V1,V2,1,minRows);
                                       //ANN TEST
                                       V2 = ann::annSVD(annV1,annV2,V2);
                                       
                                    
                                       
                                       

                                   }
                                   cloudManager.setCloud(acq::DecoratedCloud(V1,F1),0);
                                   cloudManager.setCloud(acq::DecoratedCloud(V2,F2),1);
                                   cloudManager.setCloud(acq::DecoratedCloud(V3,F3),4);
                                   cloudManager.setCloud(acq::DecoratedCloud(V4,F4),5);
                                   cloudManager.setCloud(acq::DecoratedCloud(V5,F5),6);
                                   
                                   Eigen::MatrixXd totalV(V1.rows() + V2.rows() + V3.rows()+V4.rows()+V5.rows(), V1.cols());
                                   totalV << V1, V2,V3,V4,V5;
                                   Eigen::MatrixXi totalF(F1.rows() + F2.rows()+F3.rows()+F4.rows()+F5.rows(), F1.cols());
                                   totalF << F1, (F2.array()+V1.rows()), (F3.array()+V2.rows()+V1.rows()), (F4.array()+V3.rows()+V2.rows()+V1.rows()), (F5.array()+V4.rows()+V3.rows()+V2.rows()+V1.rows());
                                   //set different colors to two objects
                                   Eigen::MatrixXd C(totalF.rows(), 3);
                                   C <<
                                   Eigen::RowVector3d(0.0, 0.3, 1.0).replicate(F1.rows(), 1),
                                   Eigen::RowVector3d(1.0, 1.0, 0.0).replicate(F2.rows(), 1),
                                   Eigen::RowVector3d(0.3, 1.0, 0.0).replicate(F3.rows(), 1),
                                   Eigen::RowVector3d(1.0, 0.0, 0.2).replicate(F4.rows(), 1),
                                   Eigen::RowVector3d(0.1, 0.4, 0.6).replicate(F5.rows(), 1);
                                   
                                   // Store read vertices and faces
                                   //                                   cloudManager.addCloud(acq::DecoratedCloud(totalV, totalF));
                                   
                                   // Show mesh
                                   viewer.data.clear();
                                   viewer.data.set_mesh(
                                                        totalV,
                                                        totalF
                                                        );
                                   
                                   viewer.data.set_colors(C);
                                   
                               }
                               );
       
        // Generate menu
        viewer.screen->performLayout();

        return false;
    }; //...viewer menu


    // Start viewer
    viewer.launch();

    return 0;
} //...main()
