#ifndef SS_SMARTFORCE_H
#define SS_SMARTFORCE_H


#include "../ISearchingStrategy.h"
#include "../include/Converters/converterPCL.h"

#include <pcl/search/flann_search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>

class ss_SmartForce : public ISearchingStrategy {

public:

    ss_SmartForce();
    ~ss_SmartForce();

    void setData(Data *d);
    void execute();

};


#endif //SS_SMARTFORCE_H
