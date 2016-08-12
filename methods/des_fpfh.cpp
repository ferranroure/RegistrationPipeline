//
// Created by yago on 16/07/12.
//

#include "des_fpfh.h"


const float normal_estimation_search_radius = 1.0f;
const float subsampling_side_length = 0.35f;
const Eigen::Vector4f subsampling_leaf_size (subsampling_side_length,
                                             subsampling_side_length,
                                             subsampling_side_length,
                                             0.0f);


void
removeOutliersDESC(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
               pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
}

void
subsampleDESC (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
           pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_subsampled)
{
    pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;
    subsampling_filter.setInputCloud (cloud);
    subsampling_filter.setLeafSize (subsampling_leaf_size);
    subsampling_filter.filter (*cloud_subsampled);
}

void
computeNormalsDESC (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
{
    cloud_normals.reset(new pcl::PointCloud<pcl::Normal> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
    normal_estimation_filter.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation_filter.setSearchMethod (search_tree);
    normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
    normal_estimation_filter.compute (*cloud_normals);
}

/*void
computeKeypointsDESC(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                 pcl::PointCloud<pcl::Normal>::Ptr &normals,
                 pcl::PointCloud<pcl::FPFHSignature33>::Ptr &features,
                 boost::shared_ptr<std::vector<int> > &keypoint_indices,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints)
{
    keypoint_indices.reset (new std::vector<int> ());

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh_estimation (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
    fpfh_estimation->setInputCloud (cloud);
    fpfh_estimation->setInputNormals (normals);

    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> feature_persistence;
    const int num_levels = 3;
    std::vector<float> scale_values(num_levels);
    float scale_begin = 0.5f * normal_estimation_search_radius;
    for (int i = 0; i < num_levels; ++i)
        scale_values[i] = ((float)(i+1.0f) * scale_begin);
    feature_persistence.setScalesVector (scale_values);

    feature_persistence.setAlpha (1.3f);
    feature_persistence.setFeatureEstimator (fpfh_estimation);
    feature_persistence.setDistanceMetric (pcl::CS);
    feature_persistence.determinePersistentFeatures (*features, keypoint_indices);

    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud (cloud);
    extract_indices_filter.setIndices (keypoint_indices);
    extract_indices_filter.filter (*keypoints);
}*/



des_fpfh::des_fpfh() {

}

des_fpfh::~des_fpfh() {

}

void des_fpfh::setData(Data *d) {
data=d;
}

void des_fpfh::execute() {

    if(data->params.realData) {
        calcDescriptors(data->A);
    }
    calcDescriptors(data->B);

}

void des_fpfh::calcDescriptors(ElementSet *X)
{

    // method parameters arbitrarily set
    // const float normal_estimation_search_radius = 1.0f;
    const int num_levels = 3;

    // Translate from our format to PCL.
    converterPCL *apcl=new converterPCL();
    PointCloud<PointXYZ>::Ptr cloud = apcl->points2PCL(X->getAllpoints());

    // remove outliers and subsample point cloud
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    removeOutliersDESC(cloud, cloud_filtered);
    cloud.swap (cloud_filtered);

    subsampleDESC(cloud, cloud_filtered);
    cloud.swap (cloud_filtered);

    // compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    computeNormalsDESC(cloud, normals);

    float radiusNormal = X->getMMD() * data->params.radiusNormalFactor;
    float radiusSearch = X->getMMD() * data->params.radiusSearchFactor;

    // compute keypoints and feature desciptors
    PointCloud<FPFHSignature33>::Ptr features (new PointCloud<FPFHSignature33>);

    PointCloud<PointXYZ>::Ptr keypoints (new PointCloud<PointXYZ>);
    boost::shared_ptr<std::vector<int> > keypoint_indices (new std::vector<int> ());

    FPFHEstimation<PointXYZ, Normal, FPFHSignature33>::Ptr fpfh_estimation (new FPFHEstimation<PointXYZ, Normal, FPFHSignature33>);
    fpfh_estimation->setInputCloud (cloud);
    fpfh_estimation->setInputNormals (normals);

    MultiscaleFeaturePersistence<PointXYZ, FPFHSignature33> feature_persistence;
    std::vector<float> scale_values(num_levels);

    float scale_begin = 0.5f * normal_estimation_search_radius;

    for (int i = 0; i < num_levels; ++i){
        scale_values[i] = ((float)(i+1.0f) * scale_begin);
    }

    feature_persistence.setScalesVector (scale_values);

    feature_persistence.setAlpha (1.3f);
    feature_persistence.setFeatureEstimator (fpfh_estimation);

    feature_persistence.setDistanceMetric (CS);
    feature_persistence.determinePersistentFeatures (*features, keypoint_indices);

    for (int i=0; i<features->size(); i++){

        FPFHSignature33 desc = features->at(i);

        vector<float> hist;

        for (int j=0; j<33; j++){

            hist.push_back((desc.histogram)[j]);
        }

        DescHistogram *myDesc = new DescHistogram(hist);

        X->setPointDescriptor(i, myDesc);
    }


}
