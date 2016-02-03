#include "det_iss.h"

det_ISS::det_ISS()
{
    data = NULL;
}

det_ISS::~det_ISS()
{}

/* SET DATA -------------------------------------------------------------
 *
 * This method sets Data variable.
 */
void det_ISS::setData(Data *d){

    data = d;
}


void det_ISS::execute(){


    computeISS(data->A);
    computeISS(data->B);

    // Aquí hariem de crear, utilizar i destruir un PCL.

    //data->A->calcDetector(data->params.detectMethod);
    //data->A->setKeypoints();
    //data->B->calcDetector(data->params.detectMethod);
    //data->B->setKeypoints();
}


void det_ISS::computeISS(ElementSet *X){

    converterPCL trans;
    PointCloud<PointXYZ>::Ptr cloud = trans.points2PCL(X->getAllpoints());


    //
    //  ISS3D parameters
    //
    double iss_salient_radius_;
    double iss_non_max_radius_;
    double iss_gamma_21_ (0.975);
    double iss_gamma_32_ (0.975);
    double iss_min_neighbors_ (5);
    int iss_threads_ (4);

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    //double model_resolution;

    // Compute model_resolution

    // Some problems with this values. Depending on that, the keypoins are too few.
    iss_salient_radius_ = 1 * X->getMMD(); //model_resolution;
    iss_non_max_radius_ = 1 * X->getMMD(); //model_resolution;

    //
    // Compute keypoints
    //
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (iss_salient_radius_);
    iss_detector.setNonMaxRadius (iss_non_max_radius_);
    iss_detector.setThreshold21 (iss_gamma_21_);
    iss_detector.setThreshold32 (iss_gamma_32_);
    iss_detector.setMinNeighbors (iss_min_neighbors_);
    iss_detector.setNumberOfThreads (iss_threads_);
    iss_detector.setInputCloud (cloud);
    iss_detector.compute (*model_keypoints);


    vector<Point*> *keypoints = new vector<Point*>;

    // Find keypoints inside our vector of points.
    if( ! model_keypoints->empty()) {

        for (int i = 0; i < model_keypoints->size(); i++) {

            Point p(model_keypoints->at(i).x, model_keypoints->at(i).y, model_keypoints->at(i).z);

            Point *x = X->findPoint(p);

            if (x != NULL) {

                keypoints->push_back(x);
            }
            else {

                cerr <<
                "det_ISS::I found differences between points from original point cloud and the keypoint vector" << endl;
                p.print();

                exit(EXIT_FAILURE);
            }
        }

        X->setWorkpoints(keypoints);

        cout << "det_ISS:: " << keypoints->size() << " keypoints have been detected" << endl;
    }
    else{
        cerr << "det_ISS::I can't found any keypoint. I going to use the entire cloud" << endl;
    }
}
