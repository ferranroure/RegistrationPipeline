/*******************************************************************************
 *  CLASS MYKDTREE (adapter class)
 *
 *  This class is a bridge between data structures from pipeline project and
 *  the external kdtree classes.
 *  The external kdtree class is:
 *      ANN: Approximate Nearest Neighbors
 *      Version: 1.1.2
 *      Release date: Jan 27, 2010
 *      David Mount and Sunil Arya (c)
 *
 *      http://www.cs.umd.edu/~mount/ANN
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef MYKDTREE_H
#define MYKDTREE_H

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <math.h>
#include "../../point.h"
#include <ANN/ANN.h>
#include "ANN/ANNperf.h"
#include "../IDataStructure.h"
#include "../../Converters/converterKdtree.h"

#define ERROR 0.0001       // Error bound


class myKdtree : public IDataStructure
{
public:


    // Elements -----------------------------------------------------------------------
    int					nPts;                                   // Current number of data points.
    ANNpointArray		dataPts;                                // Data points.
    ANNidxArray			nnIdx;                                  // Near neighbor indices.
    ANNdistArray		dists;                                  // Near neighbor distances.
    ANNkd_tree*			kdTree;                                 // Search structure.

    converterKdtree     *ckdt;


    // Methods ------------------------------------------------------------------------
    myKdtree();                                                 // Constructor.
    myKdtree(vector<Point*> *P);
    ~myKdtree();                                                // Destructor.

    returnData calcOneNN(Point *queryPoint, float errEps);      // Finds Nearest Neighbor distance to a given QueryPoint.
    returnData calcOwnNN(Point *queryPoint);                    // Finds a real NN (not itself) of a given QueryPoint from the same point cloud. (used for MMD).
    vector<returnData> calcNneigh
            (Point *queryPoint, int nNeigh);                    // Finds N nearest neighbours.
    void printStats();                                          // Print current statistics of the kdtree

};
#endif // MYKDTREE_H
