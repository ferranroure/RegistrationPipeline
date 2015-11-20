#include "mypcl.h"

/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
myPCL::myPCL(){

}


/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
myPCL::myPCL(vector<Point> *vp, float mmd){

    points = vp;
    MMD = mmd;

	cloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);

    for(vector<Point>::iterator it=vp->begin(); it!= vp->end(); ++it){

        cloud->push_back(PointXYZ(it->x, it->y, it->z));
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

}


/* DESTRUCTOR -----------------------------------------------------------
 *
 */
myPCL::~myPCL(){

}


/* UPDATE -----------------------------------------------------------
 *
 *  Updates the PCL point cloud information. Aplied after a tranformation.
 */
void myPCL::update(float mmd){

    cloud->clear();

    for(vector<Point>::iterator it=points->begin(); it!= points->end(); ++it){

        cloud->push_back(PointXYZ(it->x, it->y, it->z));
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

    MMD = mmd;

}

void myPCL::calcDescriptors(string method, float radiusNormal, float radiusSearch){


    if      (method == "SpinImage")          calcSpinImage(radiusNormal, radiusSearch);
    else if (method == "SHOT")               calcSHOT(radiusNormal, radiusSearch);
    else if (method == "FPFH")               calcFPFH(radiusNormal, radiusSearch);
    else if (method == "PrincipalCurvature") calcPrincipalCurvature(radiusNormal, radiusSearch);
    else {
        cerr << "ERROR: The method name on parameters file is not correct" << endl;
        exit(EXIT_FAILURE);
    }
}


/* CALC SHOT -----------------------------------------------------------
 *
 *  This method calculates the SHOT descriptor.
 */
void myPCL::calcSHOT(float radiusNormalFactor, float radiusSearchFactor){
 
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod (kdtree);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
    normal_estimation.setRadiusSearch (MMD*radiusNormalFactor/*0.03*/);
    normal_estimation.compute (*normals);

    //pcl::PointCloud<pcl::SHOT352>::Ptr descriptors (new pcl::PointCloud<pcl::SHOT352>());
    descriptors = PointCloud<SHOT352>::Ptr(new PointCloud<SHOT352>);

    pcl::SHOTEstimationOMP<PointXYZ, Normal, SHOT352> describer;

    describer.setRadiusSearch (MMD*radiusSearchFactor/*0.3*/);
    describer.setInputCloud (cloud);
    describer.setInputNormals (normals);
    describer.setSearchSurface (cloud);

    describer.compute (*descriptors);

    for (int i=0; i<descriptors->size(); i++){

        pcl::SHOT352 desc = descriptors->at(i);

        vector<float> hist;

        for (int j=0; j<352; j++){

            hist.push_back(desc.descriptor[j]);
        }

        (points->at(i)).setDescriptor(hist);
    }

}


/* CALC FPFH -----------------------------------------------------------
 *
 *  This method calculates the FPFH descriptor.
 */
void myPCL::calcFPFH(float radiusNormalFactor, float radiusSearchFactor){

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod (kdtree);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
    normal_estimation.setRadiusSearch (MMD*radiusNormalFactor/*0.03*/);
    normal_estimation.compute (*normals);

	// Create the FPFH estimation class, and pass the input dataset+normals to it
  	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  	fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (normals);
  	// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
  	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  	pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);

  	fpfh.setSearchMethod (tree);

  	// Output datasets
  	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  	// Use all neighbors in a sphere of radius 5cm
  	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (MMD*radiusSearchFactor);
    //fpfh.setRadiusSearch (0.03);

  	// Compute the features
  	fpfh.compute (*fpfhs);

    for (int i=0; i<fpfhs->size(); i++){

        pcl::FPFHSignature33 desc = fpfhs->at(i);

        vector<float> hist;
        for (int j=0; j<33; j++){

            hist.push_back(desc.histogram[j]);
        }

        (points->at(i)).setDescriptor(hist);
    }
}


/* CALC SPIN IMAGE -----------------------------------------------------------
 *
 *  This method calculates the Spin Image descriptor.
 */
void myPCL::calcSpinImage(float radiusNormalFactor, float radiusSearchFactor){

	// Compute the normals
  	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  	normal_estimation.setInputCloud (cloud);

  	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
  	normal_estimation.setSearchMethod (kdtree);

  	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
    normal_estimation.setRadiusSearch (MMD*radiusNormalFactor/*0.03*/);
  	normal_estimation.compute (*normals);

	// Setup spin image computation
    pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor(8, 0.5, 16);
  	spin_image_descriptor.setInputCloud (cloud);
  	spin_image_descriptor.setInputNormals (normals);

  	// Use the same KdTree from the normal estimation
  	spin_image_descriptor.setSearchMethod (kdtree);
    pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);

    spin_image_descriptor.setRadiusSearch (MMD*radiusSearchFactor);
    // spin_image_descriptor.setRadiusSearch (0.2);

  	// Actually compute the spin images
  	spin_image_descriptor.compute (*spin_images);

    for (int i=0; i<spin_images->points.size(); i++){

        pcl::Histogram<153> desc = spin_images->points[i];

        vector<float> hist;
        for (int j=0; j<153; j++){

            hist.push_back(desc.histogram[j]);
        }

        (points->at(i)).setDescriptor(hist);
    }
}


/* CALC PRINCIPAL CURVATURE -----------------------------------------------------------
 *
 *  This method calculates the Principal Curvature descriptor.
 */
void myPCL::calcPrincipalCurvature(float radiusNormalFactor, float radiusSearchFactor){

    // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

    normalEstimation.setRadiusSearch (MMD*radiusNormalFactor/*0.03*/);

    normalEstimation.compute (*cloudWithNormals);

    // Setup the principal curvatures computation
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

    // Provide the original point cloud (without normals)
    principalCurvaturesEstimation.setInputCloud (cloud);

    // Provide the point cloud with normals
    principalCurvaturesEstimation.setInputNormals(cloudWithNormals);

    // Use the same KdTree from the normal estimation
    principalCurvaturesEstimation.setSearchMethod (tree);
    principalCurvaturesEstimation.setRadiusSearch(MMD*radiusSearchFactor/*1.0*/);

    // Actually compute the principal curvatures
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principalCurvaturesEstimation.compute (*principalCurvatures);

    for (int i=0; i<principalCurvatures->points.size(); i++){

        pcl::PrincipalCurvatures desc = principalCurvatures->points[i];

        vector<float> hist;
        for (int j=0; j<3; j++){

            hist.push_back(desc.principal_curvature[j]);
        }

        (points->at(i)).setDescriptor(hist);
    }
}


void myPCL::findCorrespondences(myPCL *aux){

    CorrespondencesPtr model_scene_corrs (new Correspondences ());

    KdTreeFLANN<SHOT352> match_search;
    match_search.setInputCloud (descriptors);

      //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
      for (size_t i = 0; i < aux->descriptors->size (); ++i)
      {
        vector<int> neigh_indices (1);
        vector<float> neigh_sqr_dists (1);
        if (!pcl_isfinite (aux->descriptors->at (i).descriptor[0])) //skipping NaNs
        {
          continue;
        }
        int found_neighs = match_search.nearestKSearch (aux->descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
          Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
          model_scene_corrs->push_back (corr);
        }
      }
      cout << "Correspondences found: " << model_scene_corrs->size () << endl;
}

int myPCL::findCorrespondence(int i, myPCL *aux){

    KdTreeFLANN<SHOT352> match_search;
    match_search.setInputCloud (descriptors);

    vector<int> neigh_indices (1);
    vector<float> neigh_sqr_dists (1);

    if (!pcl_isfinite (aux->descriptors->at (i).descriptor[0])) //skipping NaNs
    {
        // do nothing
    }
    else{
        int found_neighs = match_search.nearestKSearch (aux->descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);

        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        }
    }

    //cout << "Correspondences found: index: " << neigh_indices[0] << " dist: " << neigh_sqr_dists[0] << endl;

    return neigh_indices[0];
}

myPCLReturn myPCL::findCorrespondenceVector(int i, myPCL *aux, int nPoints){


    KdTreeFLANN<SHOT352> match_search;
    match_search.setInputCloud (descriptors);


    // Create return type.
    myPCLReturn ret;


    // REVISAR AQUEST IF QUE NO VEIG CLAR EL QUE FA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (!pcl_isfinite (aux->descriptors->at(i).descriptor[0])) //skipping NaNs
    {
        // do nothing
    }
    else{
        int found_neighs = match_search.nearestKSearch (aux->descriptors->at (i), nPoints, ret.indices, ret.dists);

/*
        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        }
*/
    }


    return ret;

}












