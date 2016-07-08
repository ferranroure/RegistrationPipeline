//
// Created by ferran on 6/07/16.
//

#ifndef CONVERTERSUPER4PCS_H
#define CONVERTERSUPER4PCS_H

#include "../../external/Super4PCS/super4pcs/shared4pcs.h"
#include "../../ISearchingStrategy.h"
#include "../../include/point.h"
#include <vector>


class converterSuper4PCS {

public:

    converterSuper4PCS();
    ~converterSuper4PCS();

    vector<match_4pcs::Point3D> & convertArray(vector<Point*> *cloud, bool withNormals=false, bool withColor=false);
    motion3D * convertMatrix(cv::Mat mat);
};


#endif //PIPELINE_CONVERTERSUPER4PCS_H
