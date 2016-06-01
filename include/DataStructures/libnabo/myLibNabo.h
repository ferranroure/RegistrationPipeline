/*******************************************************************************
 *  CLASS MYLIBNABO (adapter class)
 *
 *  This class is a bridge between data structures from pipeline project and
 *  the external kdtree classes.
 *  The external kdtree class is:
 *      LibNabo:
 *      libnabo is a fast K Nearest Neighbour library for low-dimensional spaces.
 *      https://github.com/ethz-asl/libnabo
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef PIPELINE_MYLIBNABO_H
#define PIPELINE_MYLIBNABO_H

#include <iostream>
#include <vector>
#include "../IDataStructure.h"
#include "nabo/nabo.h"
#include "../../Converters/converterEigen.h"

using namespace Nabo;
using namespace Eigen;
using namespace std;


class myLibNabo : public IDataStructure {

public:

    // Elements -----------------------------------------------------

    NNSearchF * nns;
    MatrixXf dataPts;

    converterEigen * ceig;

    // Methods ------------------------------------------------------

    myLibNabo();                                             // Constructor.
    myLibNabo(vector<Point*> *P);
    ~myLibNabo();                                            // Destructor.

    returnData calcOneNN(Point *queryPoint, float errEps);                // Finds Nearest Neighbor distance to a given QueryPoint.
    returnData calcOwnNN(Point *queryPoint);                // Finds a real NN (not itself) of a given QueryPoint from the same point cloud. (used for MMD).
    vector<returnData> calcNneigh(Point *queryPoint, int nNeigh); // Finds N nearest neighbours.
    returnData findPair(Point *queryPoint, float dist);     // Find a pair for queryPoint at distance dist

    void printStats();
};


#endif //PIPELINE_MYLIBNABO_H
