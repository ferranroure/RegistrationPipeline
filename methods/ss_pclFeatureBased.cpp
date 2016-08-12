//
// Created by yago on 16/07/12.
//


#include "ss_pclFeatureBased.h"

#include <boost/format.hpp>
#include <pcl/correspondence.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>

ss_pclFeatureBased::ss_pclFeatureBased() {

}

ss_pclFeatureBased::~ss_pclFeatureBased() {

}

void ss_pclFeatureBased::setData(Data *d) {
    data = d;
}

double ss_pclFeatureBased::execute()
{

    /// REGISTRATION /////////////////////////////////////////////////////

    converterPCL conv = converterPCL();
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
    pcl::CorrespondencesPtr correspondences_filtered (new pcl::Correspondences ());

    // get the full sets
    PointCloud<PointXYZ>::Ptr target_cloud = conv.points2PCL(data->A->getAllpoints());
    PointCloud<PointXYZ>::Ptr source_cloud = conv.points2PCL(data->B->getAllpoints());

    // get the keypoingts of the two sets
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints=conv.points2PCL(data->A->workpoints);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints=conv.points2PCL(data->B->workpoints);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features=conv.desc2FPFH(data->A->workpoints);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features=conv.desc2FPFH(data->B->workpoints);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // read sensor position/orientation and then reset it
 /*   Eigen::Affine3f gt_trans_target = Eigen::Translation3f(target_cloud->sensor_origin_.head<3>()) * target_cloud->sensor_orientation_;
    Eigen::Affine3f gt_trans_source = Eigen::Translation3f(source_cloud->sensor_origin_.head<3>()) * source_cloud->sensor_orientation_;
    Eigen::Affine3f gt_trans_relative = gt_trans_target.inverse() * gt_trans_source;*/
    target_cloud->sensor_origin_.head<3>().fill(0);

    target_cloud->sensor_orientation_.setIdentity();

    source_cloud->sensor_origin_.head<3>().fill(0);
    source_cloud->sensor_orientation_.setIdentity();

    target_keypoints->sensor_origin_.head<3>().fill(0);
    target_keypoints->sensor_orientation_.setIdentity();
    source_keypoints->sensor_origin_.head<3>().fill(0);
    source_keypoints->sensor_orientation_.setIdentity();

    // original (un-registered) transform
/*    vis_transforms.push_back(Eigen::Matrix4f::Identity());
    vis_active_transform = 0;
    pcl::copyPointCloud(*source_cloud, *vis_source_cloud);
    pcl::copyPointCloud(*source_keypoints, *vis_source_keypoints);*/

    // estimate correspondences
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    // get the descriptors that should already be associated to the points, also be careful with full sets and workpoints

    est.setInputSource (source_features);
    est.setInputTarget (target_features);
    est.determineCorrespondences (*correspondences);

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector_sac;
    rejector_sac.setInputSource (source_keypoints);
    rejector_sac.setInputTarget (target_keypoints);
    rejector_sac.setInlierThreshold (2); // distance in m, not the squared distance
    rejector_sac.setMaximumIterations (1000000);
    rejector_sac.setRefineModel (false);
    rejector_sac.setInputCorrespondences(correspondences);;
    rejector_sac.getCorrespondences(*correspondences_filtered);
    correspondences.swap(correspondences_filtered);
   // std::cout << correspondences->size() << " vs. " << correspondences_filtered->size() << std::endl;
   // std::cout << "BEST TRANSFORMATION:" << std::endl << rejector_sac.getBestTransformation() << std::endl;

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
    transformation_estimation.estimateRigidTransformation (*source_keypoints,*target_keypoints,*correspondences,transform);
    std::cout << "Estimated Transform:" << std::endl << transform << std::endl;

    Eigen::Matrix4f transformInverse = transform.inverse();
    motion3D *aux = new motion3D(transformInverse);
    data->cM = aux;
    //data->cM = new motion3D(transform);





    return 0;
}
