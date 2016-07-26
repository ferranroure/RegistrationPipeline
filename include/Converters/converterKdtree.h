/*******************************************************************************
 *  CONVERTER KDTREE
 *
 *  Transforms Pipeline data types to ANN data types
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef CONVERTERKDTREE_H
#define CONVERTERKDTREE_H

#include "../point.h"
#include <ANN/ANN.h>
#include <vector>

#define DIMENSIONS 3       // Dimensions
#define ERROR 0.0001       // Error bound

class converterKdtree {

public:

    converterKdtree();                                  // Constructor
    ~converterKdtree();                                 // Destructor

    ANNpointArray convertArray(vector<Point *> *P);     // Tranforms a vector<Point> to an ANNpointArray.
    ANNpoint convertPoint(Point *p);                    // Tranforms a Point to an ANNpoint.

};
#endif //converterKdtree_H
