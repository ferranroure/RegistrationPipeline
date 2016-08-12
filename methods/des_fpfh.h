//
// Created by yago on 16/07/12.
//

#ifndef PIPELINE_DES_FPFH_H
#define PIPELINE_DES_FPFH_H


#include "../IDescription.h"
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


class des_fpfh : public IDescription {

    des_fpfh();
    ~des_fpfh();

    void setData(Data *d);
    void execute();
    void calcDescriptors(ElementSet *X);

};


#endif //PIPELINE_DES_FPFH_H
