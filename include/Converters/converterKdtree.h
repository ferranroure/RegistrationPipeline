//
// Created by ferran on 18/01/16.
// THis class is used for myKdtree
//

#ifndef CONVERTERKDTREE_H
#define CONVERTERKDTREE_H

#include "../point.h"
#include <ANN/ANN.h>
#include <vector>

#define DIMENSIONS 3       // Dimensions
#define ERROR 0.0001       // Error bound

class converterKdtree {

public:

    converterKdtree();
    ~converterKdtree();

    ANNpointArray convertArray(vector<Point *> *P);     // Tranforms a vector<Point> to an ANNpointArray.
    ANNpoint convertPoint(Point *p);                    // Tranforms a Point to an ANNpoint.

};


#endif //converterKdtree_H
