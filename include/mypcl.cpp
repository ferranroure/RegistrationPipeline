#include "mypcl.h"

/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
myPCL::myPCL(){

}

/* DESTRUCTOR -----------------------------------------------------------
 *
 */
myPCL::~myPCL(){

}

vector<Point *> * myPCL::findCorrespondences(Point *p, vector<Point *> *points, int nCorr, string method) {

    vector<Point*> *ret = new vector<Point *>();
    myPCLReturn pclRet;
    AdapterPCL apcl;

    if(method == "SHOT") {

        KdTreeFLANN<SHOT352> match_search;
        PointCloud<SHOT352>::Ptr desc = apcl.desc2SHOT(points);
        SHOT352 descPoint;
        for (int i = 0; i < p->getDescSize(); ++i) {
            descPoint.descriptor[i] = p->getDescriptor()->getValue(i);
        }

        match_search.setInputCloud(desc);
        //  if (!pcl_isfinite (aux->descSHOT->at(id).descriptor[0])) //skipping NaNs
        int found_neighs = match_search.nearestKSearch(descPoint, nCorr, pclRet.indices, pclRet.dists);
    }

    // IMPLEMENT FOR MORE METHODS.

    for (int i = 0; i < pclRet.indices.size(); ++i) {

        //cout << pclRet.indices.at(i) << " " << pclRet.dists.at(i) << endl;
        ret->push_back(points->at(pclRet.indices.at(i)));
    }

    return ret;
}





/* ------------------------------------------------------------------------------- VELL ------------------------------
///* CONSTRUCTOR -----------------------------------------------------------
// *
// */
//myPCL::myPCL(vector<Point> *vp, float mmd){
//
//    points = vp;
//    MMD = mmd;
//
//	cloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
//
//    for(vector<Point>::iterator it=vp->begin(); it!= vp->end(); ++it){
//
//        cloud->push_back(PointXYZ(it->x, it->y, it->z));
//    }
//
//    std::vector<int> indices;
//    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
//
//    workpoints = cloud;
//
//}
//
//
//
//
//
///* DESTRUCTOR -----------------------------------------------------------
// *
// *  This method switch between input cloud, using the entire cloud or the detected keypoints.
// */
//void myPCL::useKeypoints(vector<Point*> *keys){
//
//    keypoints = keys;
//
//    workpoints = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
//    if(keypoints!=NULL){
//
//        cout << "I'm using keypoints for description!" << endl;
//        for(int i=0; i<keypoints->size(); i++){
//            PointXYZ p(keypoints->at(i)->getX(), keypoints->at(i)->getY(), keypoints->at(i)->getZ());
//            workpoints->push_back(p);
//        }
//    }
//    else{
//        cout << "I'm not using keypoints for description!" << endl;
//        workpoints = cloud;
//    }
//}
//
//
///* UPDATE -----------------------------------------------------------
// *
// *  Updates the PCL point cloud information. Aplied after a tranformation.
// */
//void myPCL::update(float mmd){
//
//    cloud->clear();
//
//    for(vector<Point>::iterator it=points->begin(); it!= points->end(); ++it){
//
//        cloud->push_back(PointXYZ(it->x, it->y, it->z));
//    }
//
//    std::vector<int> indices;
//    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
//
//    MMD = mmd;
//
//    workpoints = cloud;
//
//}
//
//
///* CALC NORMALS -----------------------------------------------------------
// *
// *  This method calculates the normasl of the current point cloud.
// */
//void myPCL::calcNormals(float radiusNormalFactor){
//
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
//    normal_estimation.setInputCloud (workpoints);
//    //normal_estimation.setViewPoint(FLT_MAX, FLT_MAX, FLT_MAX);
//    //normal_estimation.setViewPoint(0,0,0);
//
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
//    normal_estimation.setSearchMethod (kdtree);
//
//    normals = pcl::PointCloud<pcl::Normal>::Ptr( new pcl::PointCloud< pcl::Normal> );
//    normal_estimation.setRadiusSearch (MMD*radiusNormalFactor);
//    //    normal_estimation.setRadiusSearch (0.03);
//    normal_estimation.compute (*normals);
//    /*
//    float vpx, vpy, vpz;
//    normal_estimation.getViewPoint(vpx, vpy, vpz);
//    repairNormalsPCL(normals, vpx, vpy, vpz);
//*/
//    repairNormals(normals);
//
//}
//
///* GET NORMALS -----------------------------------------------------------
// *
// *  Returns a vector of normals.
// */
//vector<vector3D> * myPCL::getNormals(){
//
//    vector<vector3D> * norm_v = new vector<vector3D>;
//
//    for (int i=0; i<workpoints->size(); i++){
//
//        vector3D v(normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z);
//        norm_v->push_back(v);
//    }
//
//    return norm_v;
//}
//
///* REPAIR NORMALS -----------------------------------------------------------
// *
// *  This method flips the normal vector of each point because the tranformation
// *  process have some problems with normal directions.
// */
//void myPCL::repairNormals(pcl::PointCloud<pcl::Normal>::Ptr normals){
//
//    Eigen::Vector4f c;
//    int n = pcl::compute3DCentroid(*workpoints, c);
//
//
//    Point centroid(c[0], c[1], c[2]);
//
//    int nFlip = 0;
//
//    for(int i=0; i<workpoints->size(); i++){
//
//        vector3D N(normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z);
//        Point p(workpoints->at(i).x, workpoints->at(i).y, workpoints->at(i).z);
//        // d = -ax0 -by0 -cz0;
//        float d = -(N.getX()*p.x) - (N.getY()*p.y) - ((N.getZ()*p.z));
//
//        float dist = ( (N.getX()*centroid.x) + (N.getY()*centroid.y) + (N.getZ()*centroid.z) + d );// / N.modulus() ;
//
//        if(dist > 0){
//            nFlip++;
//            normals->at(i).normal_x = (normals->at(i).normal_x)*(-1);
//            normals->at(i).normal_y = (normals->at(i).normal_y)*(-1);
//            normals->at(i).normal_z = (normals->at(i).normal_z)*(-1);
//        }
//    }
//    //cout << "HE FLIPAT " << nFlip << " VEGADES I TINC " << normals->size() << " NORMALS" <<  endl;
//
//}
//
//
///* REPAIR NORMALS PCL -----------------------------------------------------------
// *
// *  This method flips the normal vector of each point using PCL methods
// */
//void myPCL::repairNormalsPCL(pcl::PointCloud<pcl::Normal>::Ptr normals, float vpx, float vpy, float vpz){
//
//
//    for(int i=0; i<workpoints->size(); i++){
//
//        flipNormalTowardsViewpoint (workpoints->at(i), vpx, vpy, vpz, normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z);
//    }
//}
//
//
//
//
//void myPCL::calcDescriptors(string method, float radiusNormal, float radiusSearch){
//
//
//    if      (method == "SpinImage")          calcSpinImage(radiusNormal, radiusSearch);
//    else if (method == "SHOT")               calcSHOT(radiusNormal, radiusSearch);
//    else if (method == "FPFH")               calcFPFH(radiusNormal, radiusSearch);
//    else if (method == "PrincipalCurvature") calcPrincipalCurvature(radiusNormal, radiusSearch);
//    else if (method == "3DSC")               calc3DSC(radiusNormal, radiusSearch);
//    else {
//        cerr << "ERROR: The method name on parameters file is not correct" << endl;
//        exit(EXIT_FAILURE);
//    }
//}
//
//
//
///* CALC SHOT -----------------------------------------------------------
// *
// *  This method calculates the SHOT descriptor.
// */
//void myPCL::calcSHOT(float radiusNormalFactor, float radiusSearchFactor){
//
//    // check if keypoints is empty (provisional)
//
//
//    calcNormals(radiusNormalFactor);
//
//    //pcl::PointCloud<pcl::SHOT352>::Ptr descSHOT (new pcl::PointCloud<pcl::SHOT352>());
//    descSHOT = PointCloud<SHOT352>::Ptr(new PointCloud<SHOT352>);
//
//    pcl::SHOTEstimationOMP<PointXYZ, Normal, SHOT352> describer;
//
//    describer.setRadiusSearch (MMD*radiusSearchFactor);
//    //describer.setRadiusSearch(0.3);
//    describer.setInputCloud (workpoints);
//    describer.setInputNormals (normals);
//    describer.setSearchSurface (workpoints);
//    describer.compute (*descSHOT);
//
//    for (int i=0; i<descSHOT->size(); i++){
//
//        pcl::SHOT352 desc = descSHOT->at(i);
//
//        vector<float> hist;
//
//        for (int j=0; j<352; j++){
//
//            hist.push_back(desc.descriptor[j]);
//        }
//
//        DescHistogram *myDesc = new DescHistogram(hist);
//
//        (points->at(i)).setDescriptor(myDesc);
//    }
//}
//
///* CALC 3DSC -----------------------------------------------------------
// *
// *  This method calculates the 3D Shape Context descriptor.
// */
//void myPCL::calc3DSC(float radiusNormalFactor, float radiusSearchFactor){
//
//    calcNormals(radiusNormalFactor);
//
//    //pcl::PointCloud<pcl::SHOT352>::Ptr descSHOT (new pcl::PointCloud<pcl::SHOT352>());
//    desc3DSC = PointCloud<ShapeContext1980>::Ptr(new PointCloud<ShapeContext1980>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
//    pcl::ShapeContext3DEstimation<PointXYZ, Normal, ShapeContext1980> describer;
//
//    //float radius = 0.05;
//    float radius = MMD*radiusSearchFactor;
//    describer.setRadiusSearch (radius);
//    describer.setRadiusSearch(radius);
//    describer.setInputCloud (workpoints);
//    describer.setInputNormals (normals);
//    describer.setSearchSurface (workpoints);
//    describer.setSearchMethod(kdtree);
//
//    describer.setPointDensityRadius(radius/5.0f);
//    describer.setMinimalRadius(radius/10.0f);
//
//    describer.compute (*desc3DSC);
//
//    for (int i=0; i<desc3DSC->size(); i++){
//
//        pcl::ShapeContext1980 desc = desc3DSC->at(i);
//
//        vector<float> hist;
//
//        for (int j=0; j<1980; j++){
//
//            hist.push_back(desc.descriptor[j]);
//        }
//
//        DescHistogram *myDesc = new DescHistogram(hist);
//
//        (points->at(i)).setDescriptor(myDesc);
//    }
//}
//
///* CALC FPFH -----------------------------------------------------------
// *
// *  This method calculates the FPFH descriptor.
// */
//void myPCL::calcFPFH(float radiusNormalFactor, float radiusSearchFactor){
//
//    calcNormals(radiusNormalFactor);
//
//    // Output datasets
//    //pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
//    descFPFH = PointCloud<FPFHSignature33>::Ptr(new PointCloud<FPFHSignature33>);
//
//	// Create the FPFH estimation class, and pass the input dataset+normals to it
//    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> describer;
//    describer.setInputCloud (workpoints);
//    describer.setInputNormals (normals);
//  	// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);
//
//	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
//  	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//  	pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
//
//    describer.setSearchMethod (tree);
//
//
//
//  	// Use all neighbors in a sphere of radius 5cm
//  	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
//    describer.setRadiusSearch (MMD*radiusSearchFactor);
//    //fpfh.setRadiusSearch (0.03);
//
//  	// Compute the features
//    describer.compute (*descFPFH);
//
//    for (int i=0; i<descFPFH->size(); i++){
//
//        pcl::FPFHSignature33 desc = descFPFH->at(i);
//
//        vector<float> hist;
//        for (int j=0; j<33; j++){
//
//            hist.push_back(desc.histogram[j]);
//        }
//
//        DescHistogram *myDesc = new DescHistogram(hist);
//
//        (points->at(i)).setDescriptor(myDesc);
//    }
//}
//
//
///* CALC SPIN IMAGE -----------------------------------------------------------
// *
// *  This method calculates the Spin Image descriptor.
// */
//void myPCL::calcSpinImage(float radiusNormalFactor, float radiusSearchFactor){
//
//    /*// Compute the normals
//  	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
//  	normal_estimation.setInputCloud (cloud);
//
//  	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
//  	normal_estimation.setSearchMethod (kdtree);
//
//    calcNormals(radiusNormalFactor);
//    normal_estimation.setRadiusSearch (MMD*radiusNormalFactor/*0.03);
//  	normal_estimation.compute (*normals);
//
//    repairNormals(normals);
//*/
//
//    calcNormals(radiusNormalFactor);
//
//    //pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
//    descSP = PointCloud< Histogram<153> >::Ptr(new PointCloud< Histogram<153> >);
//
//	// Setup spin image computation
//    pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > describer(8, 0.5, 16);
//    describer.setInputCloud (workpoints);
//    describer.setInputNormals (normals);
//    //describer.setSearchMethod (kdtree);
//    describer.setRadiusSearch (MMD*radiusSearchFactor);
//    // describer.setRadiusSearch (0.2);
//
//  	// Actually compute the spin images
//    describer.compute (*descSP);
//
//    for (int i=0; i<descSP->points.size(); i++){
//
//        pcl::Histogram<153> desc = descSP->points[i];
//
//        vector<float> hist;
//        for (int j=0; j<153; j++){
//
//            hist.push_back(desc.histogram[j]);
//        }
//
//        DescHistogram *myDesc = new DescHistogram(hist);
//
//        (points->at(i)).setDescriptor(myDesc);
//    }
//}
//
//
///* CALC PRINCIPAL CURVATURE -----------------------------------------------------------
// *
// *  This method calculates the Principal Curvature descriptor.
// *
// *  pcx = eigenvectors_ (0, 2);
//    pcy = eigenvectors_ (1, 2);
//    pcz = eigenvectors_ (2, 2);
//    pc1 = eigenvalues_ (2);
//    pc2 = eigenvalues_ (1);
// */
//void myPCL::calcPrincipalCurvature(float radiusNormalFactor, float radiusSearchFactor){
//
//    calcNormals(radiusNormalFactor);
//
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
//
//
//    //pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
//    descPC = PointCloud<PrincipalCurvatures>::Ptr(new PointCloud<PrincipalCurvatures>);
//
//    // Setup the principal curvatures computation
//    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> describer;
//
//    // Provide the original point cloud (without normals)
//    describer.setInputCloud (workpoints);
//
//    // Provide the point cloud with normals
//    describer.setInputNormals(normals);
//
//    // Use the same KdTree from the normal estimation
//    describer.setSearchMethod (kdtree);
//    describer.setRadiusSearch(MMD*radiusSearchFactor/*1.0*/);
//
//    // Actually compute the principal curvatures
//    describer.compute (*descPC);
//
//    for (int i=0; i<descPC->points.size(); i++){
//
//        pcl::PrincipalCurvatures desc = descPC->points[i];
//
//        vector<float> hist;
//        for (int j=0; j<3; j++){
//
//            //hist.push_back(desc.principal_curvature[j]);
//        }
//
//        hist.push_back(desc.pc1);
//        hist.push_back(desc.pc2);
//
//        //(points->at(i)).setDescriptor(hist); // S'HA DE CREAR UN DescSignature : IDescriptor !!!!!!!!!!!!!!!!!!!!
//
//    }
//
//}
//
//void myPCL::findCorrespondences(myPCL *aux){
//
//    CorrespondencesPtr model_scene_corrs (new Correspondences ());
//
//    KdTreeFLANN<SHOT352> match_search;
//
//
//    match_search.setInputCloud (descSHOT);
//
//      //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
//      for (size_t i = 0; i < aux->descSHOT->size (); ++i)
//      {
//        vector<int> neigh_indices (1);
//        vector<float> neigh_sqr_dists (1);
//        if (!pcl_isfinite (aux->descSHOT->at (i).descriptor[0])) //skipping NaNs
//        {
//          continue;
//        }
//        int found_neighs = match_search.nearestKSearch (aux->descSHOT->at (i), 1, neigh_indices, neigh_sqr_dists);
//        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//        {
//          Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
//          model_scene_corrs->push_back (corr);
//        }
//      }
//      cout << "Correspondences found: " << model_scene_corrs->size () << endl;
//}
//
//// i= point id, aux is the point cloud of point i. We find inside "this" extracting point i from aux.
//int myPCL::findCorrespondence(int i, myPCL *aux){
//
//    KdTreeFLANN<SHOT352> match_search;
//
//    match_search.setInputCloud (descSHOT);
//
//    vector<int> neigh_indices (1);
//    vector<float> neigh_sqr_dists (1);
//
//    if (!pcl_isfinite (aux->descSHOT->at (i).descriptor[0])) //skipping NaNs
//    {
//        // do nothing
//    }
//    else{
//        int found_neighs = match_search.nearestKSearch (aux->descSHOT->at (i), 1, neigh_indices, neigh_sqr_dists);
//
//        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//        {
//            Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
//        }
//    }
//
//    //cout << "Correspondences found: index: " << neigh_indices[0] << " dist: " << neigh_sqr_dists[0] << endl;
//
//    return neigh_indices[0];
//}
//
//myPCLReturn myPCL::findCorrespondenceVector(int id, myPCL *aux, int nPoints, string method){
//
//    // Create return type.
//    myPCLReturn ret;
//
//    if(method == "SHOT"){
//
//        KdTreeFLANN<SHOT352> match_search;
//
//        match_search.setInputCloud (descSHOT);
//
//        // REVISAR AQUEST IF QUE NO VEIG CLAR EL QUE FA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//        if (!pcl_isfinite (aux->descSHOT->at(id).descriptor[0])) //skipping NaNs
//        {
//            // do nothing
//        }
//        else{
//            int found_neighs = match_search.nearestKSearch (aux->descSHOT->at (id), nPoints, ret.indices, ret.dists);
//        }
//    }
//    else if(method == "FPFH"){
//
//
//        KdTreeFLANN<FPFHSignature33> match_search;
//
//        match_search.setInputCloud (descFPFH);
//
//        // REVISAR AQUEST IF QUE NO VEIG CLAR EL QUE FA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//        if (!pcl_isfinite (aux->descFPFH->at(id).histogram[0])) //skipping NaNs
//        {
//            // do nothing
//        }
//        else{
//            int found_neighs = match_search.nearestKSearch (aux->descFPFH->at (id), nPoints, ret.indices, ret.dists);
//        }
//
//    }
//    else if(method == "3DSC"){
//
//        KdTreeFLANN<ShapeContext1980> match_search;
//
//        match_search.setInputCloud (desc3DSC);
//
//        // REVISAR AQUEST IF QUE NO VEIG CLAR EL QUE FA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//        if (!pcl_isfinite (aux->desc3DSC->at(id).descriptor[0])) //skipping NaNs
//        {
//            // do nothing
//        }
//        else{
//            int found_neighs = match_search.nearestKSearch (aux->desc3DSC->at (id), nPoints, ret.indices, ret.dists);
//        }
//
//    }
//
//
//    return ret;
//}
//
//Point myPCL::calcCentroid(){
//
//    Eigen::Vector4f c;
//    int n = pcl::compute3DCentroid(*workpoints, c);
//
//    return Point(c[0], c[1], c[2]);
//}
//
//Eigen::Matrix3f * myPCL::calcCovarianceMatrix(){
//
//    Eigen::Matrix3f *cv = new Eigen::Matrix3f();
//
//    Eigen::Vector4f c;
//    compute3DCentroid(*workpoints, c);
//
//    computeCovarianceMatrixNormalized(*workpoints, c, *cv);
//
//    return cv;
//}
//
//
//
//// ----------------------- DETECTORS ----------------------------------------------
//
//vector<Point> * myPCL::calcDetectors(string method){
//
//    vector<Point> * ret;
//
//    if      (method == "ISS")          ret = calcISS();
//    else {
//        cerr << "ERROR: The detector method name on parameters file is not correct" << endl;
//        exit(EXIT_FAILURE);
//    }
//
//    return ret;
//
//}
//
//
//vector<Point>* myPCL::calcISS(){
//
//    //
//    //  ISS3D parameters
//    //
//    double iss_salient_radius_;
//    double iss_non_max_radius_;
//    double iss_gamma_21_ (0.975);
//    double iss_gamma_32_ (0.975);
//    double iss_min_neighbors_ (5);
//    int iss_threads_ (4);
//
//    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA> ());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//
//    //double model_resolution;
//
//    // Compute model_resolution
//
//    iss_salient_radius_ = 6 * MMD; //model_resolution;
//    iss_non_max_radius_ = 4 * MMD; //model_resolution;
//
//    //
//    // Compute keypoints
//    //
//    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
//
//    iss_detector.setSearchMethod (tree);
//    iss_detector.setSalientRadius (iss_salient_radius_);
//    iss_detector.setNonMaxRadius (iss_non_max_radius_);
//    iss_detector.setThreshold21 (iss_gamma_21_);
//    iss_detector.setThreshold32 (iss_gamma_32_);
//    iss_detector.setMinNeighbors (iss_min_neighbors_);
//    iss_detector.setNumberOfThreads (iss_threads_);
//    iss_detector.setInputCloud (cloud);
//    iss_detector.compute (*model_keypoints);
//
//
//    return updateElementSet(model_keypoints);
//}
//
//vector<Point> * myPCL::updateElementSet(pcl::PointCloud<pcl::PointXYZ>::Ptr keys){
//
//    vector<Point> * ret = new vector<Point>;
//
//    for(int i=0; i<keys->size(); i++){
//
//        ret->push_back(Point(keys->at(i).x, keys->at(i).y, keys->at(i).z));
//    }
//
//    return ret;
//
//}
//
//
//


