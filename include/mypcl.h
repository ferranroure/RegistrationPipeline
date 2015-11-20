/*******************************************************************************
 *  CLASS MYPCL
 *
 *  This class is a bridge between Point Cloud Library and our Pipelnie
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <vector>
#include "point.h"
#include "descHistogram.h"
#include "AdapterPCL.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/point_representation.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/keypoints/iss_3d.h>

#include <pcl/filters/filter.h>
#include <pcl/search/flann_search.h>

//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>


#ifndef MYPCL_H
#define MYPCL_H


using namespace std;
using namespace pcl;

//// Data structure used to return different data, out of this class.
//struct myPCLReturn{
//
//    vector<int> indices;
//    vector<float> dists;
//};


class myPCL
{
public:

    // Methods ---- -----------------------------------------------------------
    myPCL();                                            // Constructor.
    ~myPCL();
    vector<Point*> * findCorrespondences(Point *p, vector<Point*> *points, int nCorr, string method);


/* VELL
*
    // Elements --------------------------------------------------------------
    PointCloud<PointXYZ>::Ptr cloud;                    // PCL point cloud.
    PointCloud<PointXYZ>::Ptr workpoints;

    PointCloud<SHOT352>::Ptr descSHOT;
    PointCloud< pcl::Histogram<153> >::Ptr descSP;
    PointCloud<FPFHSignature33>::Ptr descFPFH;
    PointCloud<PrincipalCurvatures>::Ptr descPC;
    PointCloud<ShapeContext1980>::Ptr desc3DSC;
    PointCloud<Normal>::Ptr normals;
    vector<Point> *points;                              // Pointer to a Point Vector of the model.
    vector<Point*> *keypoints;
    float MMD;                                          // Mean Minimum Distance of points.
	

    // Methods ---- -----------------------------------------------------------
    myPCL();                                            // Constructor.
    myPCL(vector<Point> *vp, float mmd);                    // Constructor.
    ~myPCL();                                           // Destructor.

    void useKeypoints(vector<Point*> *keys);

    void update(float mmd);                             // Update
    void calcNormals(float radiusNormalFactor);
    vector<vector3D> * getNormals();
    void repairNormals(pcl::PointCloud<pcl::Normal>::Ptr normals);
    void repairNormalsPCL(pcl::PointCloud<pcl::Normal>::Ptr normals, float vpx, float vpy, float vpz);

    void calcDescriptors(string method, float radiusNormal, float radiusSearch);
    void calcSHOT(float radiusNormal, float radiusSearch);
    void calcFPFH(float radiusNormal, float radiusSearch);
    void calcSpinImage(float radiusNormal, float radiusSearch);
    void calcPrincipalCurvature(float radiusNormal, float radiusSearch);
    void calc3DSC(float radiusNormal, float radiusSearch);

    Point calcCentroid();
    Eigen::Matrix3f *calcCovarianceMatrix();

    void findCorrespondences(myPCL *aux);
    int findCorrespondence(int i, myPCL *aux);
    myPCLReturn findCorrespondenceVector(int i, myPCL *aux, int nPoints, string method);

    // DETECTORS

    vector<Point> *calcDetectors(string method);
    vector<Point> *calcISS();
    vector<Point> *updateElementSet(pcl::PointCloud<pcl::PointXYZ>::Ptr keys);
*/
};

#endif // MYPCL_H
