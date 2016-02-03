#ifndef CONVERSOR4PCS_H
#define CONVERSOR4PCS_H

#include "../4pcsRegistrationObject.h"
#include "../point.h"
#include "../../external/motion3D.h"
#include <vector>

class converter4PCS {

public:
    converter4PCS();
    ~converter4PCS();

    vector<Point3D> *points24PCS(vector<Point *> *cloud, bool withNormals, bool withColor);
    motion3D * mat2motion(double mat[4][4]);

};


#endif //ADAPTER4PCS_H
