#include "acq/normalEstimation.h"
#include "acq/decoratedCloud.h"
#include "acq/cloudManager.h"

#include "nanogui/formhelper.h"
#include "nanogui/screen.h"

#include "igl/readOFF.h"
#include "igl/viewer/Viewer.h"

#include <iostream>
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
    int kNeighbours = 10;
    // Maximum distance between vertices to be considered neighbours (FLANN mode)
    float maxNeighbourDist = 0.15; //TODO: set to average vertex distance upon read

    // Dummy enum to demo GUI
    enum Orientation { Up=0, Down, Left, Right } dir = Up;
    // Dummy variable to demo GUI
    bool boolVariable = true;
    // Dummy variable to demo GUI
    float floatVariable = 0.1f;
	/*Eigen::Matrix<float, 3, 1> T = Eigen::Matrix<float, 3, 1>::Identity();
	T(0, 0) = 10.0f;
	T(1, 0) = 1.0f;
	T(2, 0) = 0.0f;*/
	//Eigen::Affine3f transform(Eigen::Translation3f(10, 2, 3));
	//Eigen::Matrix4f T = transform.matrix();

	/*std::vector<float> T(3, 1);
	T = { 10.0f, 5.0f, 0.0f };*/
//    Eigen::Affine3f transform(Eigen::Translation3f(0.1f,0.0f,0.3f));

	Eigen::Vector3d T;
	T << 0.1f, 0.0f, 0.0f;
	Eigen::Matrix3d R;
	R << 0, 0, 1, 0, 1, 0, 1, 0, 0;
//    R << 0, 0, 1, 0, 1, 0, 1, 0, 0;
    // Load a mesh in OFF format
    std::string meshPath = "../3rdparty/libigl/tutorial/shared/bunny.off";
	std::string newMesh = "../3rdparty/libigl/tutorial/shared/camelhead.off";
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
		
        // Face indices, M x 3 integers referring to V.
        Eigen::MatrixXi F;
		Eigen::MatrixXi newF;
        // Read mesh
        igl::readOFF(meshPath, V1, F);
		if (V1.rows() <= 0) {
			std::cerr << "Could not read mesh at " << meshPath
				<< "...exiting...\n";
			return EXIT_FAILURE;
		} //...if vertices read
        //----------------Point Processing-----------//
        //        Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
        //// 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
        //Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) );     //沿 Z 轴旋转 45 度
        //
        //rotation_matrix = rotation_vector.toRotationMatrix();
        //Eigen::Isometry3d T=Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
        //T.rotate ( rotation_vector );                                     // 按照rotation_vector进行旋转
        //T.pretranslate ( Eigen::Vector3d ( 0.1f,0.0,0.2f ) );                     // 把平移向量设成(1,3,4)
        //cout << "Transform matrix = \n" << T.matrix() <<endl;
        
        float v1XMean =V1.col(0).mean();
        float v1YMean =V1.col(1).mean();
        float v1ZMean =V1.col(2).mean();
        
        cout<<"oldMean of V1 X col0 is: "<<v1XMean<<endl;
        cout<<"oldMean of V1 Y col1 is: "<<v1YMean<<endl;
        cout<<"oldMean of V1 Z col2 is: "<<v1ZMean<<endl;
		cout << endl;
        V2 = (V1 + T.replicate<1, 3485>().transpose());
        //V2 = T*V1;
        float v2XMean =V2.col(0).mean();
        float v2YMean =V2.col(1).mean();
        float v2ZMean =V2.col(2).mean();
        cout<<"oldMean of V2 X col0 is: "<<v2XMean<<endl;
        cout<<"oldMean of V2 Y col1 is: "<<v2YMean<<endl;
        cout<<"oldMean of V2 Z col2 is: "<<v2ZMean<<endl;
		cout << endl;
		Eigen::Vector3d v1Mean;
		v1Mean << v1XMean, v1YMean, v1ZMean;
		Eigen::Vector3d v2Mean;
		v2Mean << v2XMean, v2YMean, v2ZMean;
		Eigen::MatrixXd newV1;
		Eigen::MatrixXd newV2;
		newV1 = V1 - v1Mean.replicate<1, 3485>().transpose();
		newV2 = V2 - v2Mean.replicate<1, 3485>().transpose();
		
		cloudManager.addCloud(acq::DecoratedCloud(newV1, F));
		cloudManager.addCloud(acq::DecoratedCloud(newV2, F));
		acq::NeighboursT const SVDneighbours =
			acq::calculateCloudNeighbours(
				/* [in] Source Cloud: */ cloudManager.getCloud(0).getVertices(),
				/* [in] Target Cloud: */ cloudManager.getCloud(1).getVertices(),
				/* [in] k-neighbours: */ 1,
				/* [in]      maxDist: */ maxNeighbourDist
			);
		cout<<"Neighbours size: "<<SVDneighbours.size()<<endl;
		Eigen::MatrixXd V3; //neighbour Vs
		
		V3.setZero(SVDneighbours.size(),3);
		for (int i = 0; i < SVDneighbours.size(); i++)
		{
			std::set<size_t> neighbours = SVDneighbours.at(i);
			std::set<size_t>::iterator eachN;
			for (eachN = neighbours.begin(); eachN != neighbours.end(); eachN++) {
				//cout <<"Number of Point: "<<i <<" "<<V1.row(*eachN) <<endl;
				V3.row(i) = newV1.row(*eachN);
			}
		}

		cout << "Mean of V1 X col0 is: " << newV1.col(0).mean() << endl;
		cout << "Mean of V1 Y col1 is: " << newV1.col(1).mean() << endl;
		cout << "Mean of V1 Z col2 is: " << newV1.col(2).mean() << endl;
		cout <<endl;
		float v3XMean = V3.col(0).mean();
		float v3YMean = V3.col(1).mean();
		float v3ZMean = V3.col(2).mean();
		Eigen::Vector3d v3Mean;
		v3Mean << v3XMean, v3YMean, v3ZMean;
		cout << "Mean of V3 X col0 is: " << v3XMean << endl;
		cout << "Mean of V3 Y col1 is: " << v3YMean << endl;
		cout << "Mean of V3 Z col2 is: " << v3ZMean << endl;
		cout << endl;
		float newv2XMean = newV2.col(0).mean();
		float newv2YMean = newV2.col(1).mean();
		float newv2ZMean = newV2.col(2).mean();
		cout << "Mean of V2 X col0 is: " << newv2XMean << endl;
		cout << "Mean of V2 Y col1 is: " << newv2YMean << endl;
		cout << "Mean of V2 Z col2 is: " << newv2ZMean << endl;
		cout << endl;
		Eigen::Vector3d newv2Mean;
		newv2Mean << newv2XMean, newv2YMean, newv2ZMean;

		Eigen::MatrixXd SVDC;
		SVDC = ((newV2 - newv2Mean.replicate<1, 3485>().transpose()).transpose())*(V3 - v3Mean.replicate<1, 3485>().transpose());
		cout << "The SVDC is"<<endl << SVDC << endl;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(SVDC, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXd SVDU = svd.matrixU();
		Eigen::MatrixXd SVDV = svd.matrixV();
		Eigen::MatrixXd SVDR = SVDV*SVDU.transpose();
		Eigen::MatrixXd SVDT = v3Mean - SVDR*newv2Mean;
		cout << "The SVDR is" << endl << SVDR << endl;
		cout << "The SVDT is" << endl << SVDT << endl;
		//V2 = (V2 - SVDT.replicate<1, 3485>().transpose())*SVDR;

        //----------------Point Processing-----------//
		//----------------Show Two Meshes-----------//
        newF = F;
		
//		V2 = (V1 + T.replicate<1, 3485>().transpose())*R;
		
        //igl::readOFF(newMesh, newV, newF);

		Eigen::MatrixXd totalV(V1.rows() + V2.rows(), V1.cols());
		totalV << V1, V2;
		Eigen::MatrixXi totalF(F.rows() + newF.rows(), F.cols());
		totalF << F, (newF.array() + V1.rows());

		//set different colors to two objects
		Eigen::MatrixXd C(totalF.rows(), 3);
		C <<
			Eigen::RowVector3d(0.2, 0.3, 0.8).replicate(F.rows(), 1),
			Eigen::RowVector3d(1.0, 0.7, 0.2).replicate(newF.rows(), 1);

        // Store read vertices and faces
        cloudManager.addCloud(acq::DecoratedCloud(V1, F));
		
        // Show mesh
		viewer.data.set_mesh(
			totalV,
            totalF
        );
		
		viewer.data.set_colors(C);
		// Calculate normals on launch
		cloudManager.getCloud(0).setNormals(
			acq::recalcNormals(
				/* [in]      K-neighbours for FLANN: */ kNeighbours,
				/* [in]             Vertices matrix: */ cloudManager.getCloud(0).getVertices(),
				/* [in]      max neighbour distance: */ maxNeighbourDist
			)
		);

		// Update viewer
		acq::setViewerNormals(
			viewer,
			cloudManager.getCloud(0).getVertices(),
			cloudManager.getCloud(0).getNormals()
		);
		
		
		

    } //...read mesh

	

    // Extend viewer menu using a lambda function
    viewer.callback_init =
        [
            &cloudManager, &kNeighbours, &maxNeighbourDist,
            &floatVariable, &boolVariable, &dir
        ] (igl::viewer::Viewer& viewer)
    {
        // Add an additional menu window
        viewer.ngui->addWindow(Eigen::Vector2i(900,10), "Acquisition3D");

        // Add new group
        viewer.ngui->addGroup("Nearest neighbours (pointcloud, FLANN)");

        // Add k-neighbours variable to GUI
        viewer.ngui->addVariable<int>(
            /* Displayed name: */ "k-neighbours",

            /*  Setter lambda: */ [&] (int val) {
                // Store reference to current cloud (id 0 for now)
                acq::DecoratedCloud &cloud = cloudManager.getCloud(0);

                // Store new value
                kNeighbours = val;

                // Recalculate normals for cloud and update viewer
                cloud.setNormals(
                    acq::recalcNormals(
                        /* [in]      K-neighbours for FLANN: */ kNeighbours,
                        /* [in]             Vertices matrix: */ cloud.getVertices(),
                        /* [in]      max neighbour distance: */ maxNeighbourDist
                    )
                );

                // Update viewer
                acq::setViewerNormals(
                    /* [in, out] Viewer to update: */ viewer,
                    /* [in]            Pointcloud: */ cloud.getVertices(),
                    /* [in] Normals of Pointcloud: */ cloud.getNormals()
                );
            }, //...setter lambda

            /*  Getter lambda: */ [&]() {
                return kNeighbours; // get
            } //...getter lambda
        ); //...addVariable(kNeighbours)

        // Add maxNeighbourDistance variable to GUI
        viewer.ngui->addVariable<float>(
            /* Displayed name: */ "maxNeighDist",

            /*  Setter lambda: */ [&] (float val) {
                // Store reference to current cloud (id 0 for now)
                acq::DecoratedCloud &cloud = cloudManager.getCloud(0);

                // Store new value
                maxNeighbourDist = val;

                // Recalculate normals for cloud and update viewer
                cloud.setNormals(
                    acq::recalcNormals(
                        /* [in]      K-neighbours for FLANN: */ kNeighbours,
                        /* [in]             Vertices matrix: */ cloud.getVertices(),
                        /* [in]      max neighbour distance: */ maxNeighbourDist
                    )
                );

                // Update viewer
                acq::setViewerNormals(
                    /* [in, out] Viewer to update: */ viewer,
                    /* [in]            Pointcloud: */ cloud.getVertices(),
                    /* [in] Normals of Pointcloud: */ cloud.getNormals()
                );
            }, //...setter lambda

            /*  Getter lambda: */ [&]() {
                return maxNeighbourDist; // get
            } //...getter lambda
        ); //...addVariable(kNeighbours)

        // Add a button for estimating normals using FLANN as neighbourhood
        // same, as changing kNeighbours
        viewer.ngui->addButton(
            /* displayed label: */ "Estimate normals (FLANN)",

            /* lambda to call: */ [&]() {
                // store reference to current cloud (id 0 for now)
                acq::DecoratedCloud &cloud = cloudManager.getCloud(0);

                // calculate normals for cloud and update viewer
                cloud.setNormals(
                    acq::recalcNormals(
                        /* [in]      k-neighbours for flann: */ kNeighbours,
                        /* [in]             vertices matrix: */ cloud.getVertices(),
                        /* [in]      max neighbour distance: */ maxNeighbourDist
                    )
                );

                // update viewer
                acq::setViewerNormals(
                    /* [in, out] viewer to update: */ viewer,
                    /* [in]            pointcloud: */ cloud.getVertices(),
                    /* [in] normals of pointcloud: */ cloud.getNormals()
                );
            } //...button push lambda
        ); //...estimate normals using FLANN

        // Add a button for orienting normals using FLANN
        viewer.ngui->addButton(
            /* Displayed label: */ "Orient normals (FLANN)",

            /* Lambda to call: */ [&]() {
                // Store reference to current cloud (id 0 for now)
                acq::DecoratedCloud &cloud = cloudManager.getCloud(0);

                // Check, if normals already exist
                if (!cloud.hasNormals())
                    cloud.setNormals(
                        acq::recalcNormals(
                            /* [in]      K-neighbours for FLANN: */ kNeighbours,
                            /* [in]             Vertices matrix: */ cloud.getVertices(),
                            /* [in]      max neighbour distance: */ maxNeighbourDist
                        )
                    );

                // Estimate neighbours using FLANN
                acq::NeighboursT const neighbours =
                    acq::calculateCloudNeighbours(
                        /* [in]        Cloud: */ cloud.getVertices(),
                        /* [in] k-neighbours: */ kNeighbours,
                        /* [in]      maxDist: */ maxNeighbourDist
                    );

                // Orient normals in place using established neighbourhood
                int nFlips =
                    acq::orientCloudNormals(
                        /* [in    ] Lists of neighbours: */ neighbours,
                        /* [in,out]   Normals to change: */ cloud.getNormals()
                    );
                std::cout << "nFlips: " << nFlips << "/" << cloud.getNormals().size() << "\n";

                // Update viewer
                acq::setViewerNormals(
                    /* [in, out] Viewer to update: */ viewer,
                    /* [in]            Pointcloud: */ cloud.getVertices(),
                    /* [in] Normals of Pointcloud: */ cloud.getNormals()
                );
            } //...lambda to call on buttonclick
        ); //...addButton(orientFLANN)


        // Add new group
        viewer.ngui->addGroup("Connectivity from faces ");

        // Add a button for estimating normals using faces as neighbourhood
        viewer.ngui->addButton(
            /* Displayed label: */ "Estimate normals (from faces)",

            /* Lambda to call: */ [&]() {
                // Store reference to current cloud (id 0 for now)
                acq::DecoratedCloud &cloud = cloudManager.getCloud(0);

                // Check, if normals already exist
                if (!cloud.hasNormals())
                    cloud.setNormals(
                        acq::recalcNormals(
                            /* [in]      K-neighbours for FLANN: */ kNeighbours,
                            /* [in]             Vertices matrix: */ cloud.getVertices(),
                            /* [in]      max neighbour distance: */ maxNeighbourDist
                        )
                    );

                // Estimate neighbours using FLANN
                acq::NeighboursT const neighbours =
                    acq::calculateCloudNeighboursFromFaces(
                        /* [in] Faces: */ cloud.getFaces()
                    );

                // Estimate normals for points in cloud vertices
                cloud.setNormals(
                    acq::calculateCloudNormals(
                        /* [in]               Cloud: */ cloud.getVertices(),
                        /* [in] Lists of neighbours: */ neighbours
                    )
                );

                // Update viewer
                acq::setViewerNormals(
                    /* [in, out] Viewer to update: */ viewer,
                    /* [in]            Pointcloud: */ cloud.getVertices(),
                    /* [in] Normals of Pointcloud: */ cloud.getNormals()
                );
            } //...button push lambda
        ); //...estimate normals from faces

        // Add a button for orienting normals using face information
        viewer.ngui->addButton(
            /* Displayed label: */ "Orient normals (from faces)",

            /* Lambda to call: */ [&]() {
                // Store reference to current cloud (id 0 for now)
                acq::DecoratedCloud &cloud = cloudManager.getCloud(0);

                // Check, if normals already exist
                if (!cloud.hasNormals())
                    cloud.setNormals(
                        acq::recalcNormals(
                            /* [in]      K-neighbours for FLANN: */ kNeighbours,
                            /* [in]             Vertices matrix: */ cloud.getVertices(),
                            /* [in]      max neighbour distance: */ maxNeighbourDist
                        )
                    );

                // Orient normals in place using established neighbourhood
                int nFlips =
                    acq::orientCloudNormalsFromFaces(
                        /* [in    ] Lists of neighbours: */ cloud.getFaces(),
                        /* [in,out]   Normals to change: */ cloud.getNormals()
                    );
                std::cout << "nFlips: " << nFlips << "/" << cloud.getNormals().size() << "\n";

                // Update viewer
                acq::setViewerNormals(
                    /* [in, out] Viewer to update: */ viewer,
                    /* [in]            Pointcloud: */ cloud.getVertices(),
                    /* [in] Normals of Pointcloud: */ cloud.getNormals()
                );
            } //...lambda to call on buttonclick
        ); //...addButton(orientFromFaces)


        // Add new group
        viewer.ngui->addGroup("Util");

        // Add a button for flipping normals
        viewer.ngui->addButton(
            /* Displayed label: */ "Flip normals",
            /*  Lambda to call: */ [&](){
                // Store reference to current cloud (id 0 for now)
                acq::DecoratedCloud &cloud = cloudManager.getCloud(0);

                // Flip normals
                cloud.getNormals() *= -1.f;

                // Update viewer
                acq::setViewerNormals(
                    /* [in, out] Viewer to update: */ viewer,
                    /* [in]            Pointcloud: */ cloud.getVertices(),
                    /* [in] Normals of Pointcloud: */ cloud.getNormals()
                );
            } //...lambda to call on buttonclick
        );

        // Add a button for setting estimated normals for shading
        viewer.ngui->addButton(
            /* Displayed label: */ "Set shading normals",
            /*  Lambda to call: */ [&](){

                // Store reference to current cloud (id 0 for now)
                acq::DecoratedCloud &cloud = cloudManager.getCloud(0);

                // Set normals to be used by viewer
                viewer.data.set_normals(cloud.getNormals());

            } //...lambda to call on buttonclick
        );

        // ------------------------
        // Dummy libIGL/nanoGUI API demo stuff:
        // ------------------------

        // Add new group
        viewer.ngui->addGroup("Dummy GUI demo");

        // Expose variable directly ...
        viewer.ngui->addVariable("float", floatVariable);

        // ... or using a custom callback
        viewer.ngui->addVariable<bool>(
            "bool",
            [&](bool val) {
                boolVariable = val; // set
            },
            [&]() {
                return boolVariable; // get
            }
        );

        // Expose an enumaration type
        viewer.ngui->addVariable<Orientation>("Direction",dir)->setItems(
            {"Up","Down","Left","Right"}
        );

        // Add a button
        viewer.ngui->addButton("Print Hello",[]() {
            std::cout << "Hello\n";
        });

        // Generate menu
        viewer.screen->performLayout();

        return false;
    }; //...viewer menu


    // Start viewer
    viewer.launch();

    return 0;
} //...main()
