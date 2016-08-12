//
// Created by yago on 16/07/12.
//

#ifndef PIPELINE_SS_PCLFEATUREBASED_H
#define PIPELINE_SS_PCLFEATUREBASED_H

#include "../ISearchingStrategy.h"
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>


class ss_pclFeatureBased : public ISearchingStrategy{




public:

    ss_pclFeatureBased();
    ~ss_pclFeatureBased();

    void setData(Data *d);
    double execute();
};


#endif //PIPELINE_SS_PCLFEATUREBASED_H
