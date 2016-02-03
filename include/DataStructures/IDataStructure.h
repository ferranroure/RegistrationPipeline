/*******************************************************************************
 *  Interface IDATASTRUCTURE
 *
 *  This interface defines a contract that must be followed to use a external
    data structure for any kind of search of point organization.

 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef IDATASTRUCTURE_H
#define IDATASTRUCTURE_H

#include <iostream>
#include <vector>
#include "../point.h"


struct returnData{

    int index;
    double sqrDist;
};


class IDataStructure
{


public:

    IDataStructure(){}
    virtual ~IDataStructure(){}
//    virtual void create(vector<Point*> *P) = 0;
//    virtual void setData(vector<Point*> *data) = 0;
    virtual returnData calcOneNN(Point *queryPoint, float errEps) = 0;        // Finds Nearest Neighbor distance to a given QueryPoint.
    virtual returnData calcOwnNN(Point *queryPoint) = 0;        // Finds a real NN (not itself) of a given QueryPoint from the same point cloud. (used for MMD)
    virtual vector<returnData>  calcNneigh(Point *queryPoint, int nNeigh) = 0;
};

#endif // IDATASTRUCTURE_H
