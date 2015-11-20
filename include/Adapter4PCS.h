#ifndef ADAPTER4PCS_H
#define ADAPTER4PCS_H

#include "4pcsRegistrationObject.h"
#include "point.h"
#include "../external/motion3D.h"
#include <vector>

class Adapter4PCS {

public:
    Adapter4PCS();
    ~Adapter4PCS();

    vector<Point3D> *points24PCS(vector<Point *> *cloud, bool withNormals, bool withColor);
    motion3D * mat2motion(double mat[4][4]);

};


#endif //ADAPTER4PCS_H
