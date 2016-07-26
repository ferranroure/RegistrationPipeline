/*******************************************************************************
 *  Interface IDATASTRUCTURE
 *
 *  This interface defines a contract that must be followed to use a external
 *  data structure used in the Pipeline.
 *  In order to include a new data structure, two classes must be implemented:
 *      - Adapter class (myAmazingDS)
 *          This adapter should implement this interface
 *          using the methods of the new data structure.
 *      - Converter class (ConverterAmazingDS)
 *          This class is used to transform the data types from the Pipeline
 *          to the custom data types from the new data structure and viceversa.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef IDATASTRUCTURE_H
#define IDATASTRUCTURE_H

#define DIMENSIONS 3       // Dimensions

#include <iostream>
#include <vector>
#include "../point.h"

// This structure is used as a return type the methods.
struct returnData{

    int index;      // Index of the NN point.
    double sqrDist; // Distance from each point to the query point.
};


class IDataStructure
{
public:

    IDataStructure(){}                          // Constructor
    virtual ~IDataStructure(){}                 // Destructor
    virtual returnData calcOneNN
            (Point *queryPoint,
             float errEps) = 0;                 // Find Nearest Neighbor distance to a given QueryPoint.
    virtual returnData calcOwnNN
            (Point *queryPoint) = 0;            // Find a real NN (not itself) of a given QueryPoint from the same point cloud. (used for MMD)
    virtual vector<returnData>  calcNneigh
            (Point *queryPoint,
             int nNeigh) = 0;                   // Find a set of nearest neigbours of a query point.

    virtual void printStats() = 0;              // Print data structure statistics.
};

#endif // IDATASTRUCTURE_H
