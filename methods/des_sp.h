#ifndef DES_SP_H
#define DES_SP_H

#include "../IDescription.h"
#include "../include/Converters/converterPCL.h"

#include <pcl/search/flann_search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>

class des_SP : public IDescription
{
public:
    des_SP();
    ~des_SP();

    void setData(Data *d);
    void execute();
    void calcDescriptors(ElementSet *X);

};


#endif //DES_SP_H
