//
// Created by ferran on 27/05/16.
//

#ifndef PIPELINE_CONVERTEREIGEN_H
#define PIPELINE_CONVERTEREIGEN_H

#include "../point.h"
#include "nabo/nabo.h"
#include <vector>

using namespace Nabo;
using namespace Eigen;

class converterEigen {

public:

    converterEigen();
    ~converterEigen();

    MatrixXf convertArray(vector<Point *> *P);
    VectorXf convertPoint(Point *p);
};


#endif //PIPELINE_CONVERTEREIGEN_H
