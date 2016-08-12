//
// Created by yago on 16/07/08.
//

#include "../IDetection.h"
//#include "../include/Converters/converterPCL.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/filters/extract_indices.h>

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

//#include <pcl/search/flann_search.h>
//#include <pcl/keypoints/iss_3d.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
//#include <pcl/point_cloud.h>

#ifndef PIPELINE_DET_H
#define PIPELINE_DET_H


class det_FPFHSignature33 : public IDetection
{


public:
    det_FPFHSignature33();
    ~det_FPFHSignature33();

    void setData(Data *d);
    void execute();
    void calcDescriptors(ElementSet *X);


};


#endif //PIPELINE_DET_H
