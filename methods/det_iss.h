#ifndef DET_ISS_H
#define DET_ISS_H

#include "../IDetection.h"
#include "../include/AdapterPCL.h"

#include <pcl/search/flann_search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>



class det_ISS : public IDetection
{
public:
    det_ISS();
    ~det_ISS();

    void setData(Data *d);
    void execute();

    void computeISS(ElementSet *X);
};

#endif // DET_ISS_H
