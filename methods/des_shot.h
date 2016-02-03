#ifndef DES_SHOT_H
#define DES_SHOT_H

#include "../IDescription.h"
#include "../include/Converters/converterPCL.h"

#include <pcl/search/flann_search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>



class des_SHOT : public IDescription
{
public:
    des_SHOT();
    ~des_SHOT();

    void setData(Data *d);
    void execute();
    void calcDescriptors(ElementSet *X);

};

#endif // DET_SHOT_H
