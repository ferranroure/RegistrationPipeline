/*******************************************************************************
 *  Abstract Class ISearching Strategy
 *
 *  Contract of Searching Strategies methods.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef ISEARCHINGSTRATEGY_H
#define ISEARCHINGSTRATEGY_H

#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
#include "data.h"
#include "include/mypcl.h"
#include "include/elementset.h"
#include "include/point.h"
#include "external/motion3D.h"
#include "include/base.h"

class ISearchingStrategy
{
protected:
    // Elements -------------------------------------------------
    Data *data;

public:

    // Methods ------------------------------------------------

    ISearchingStrategy();
    ~ISearchingStrategy();

    virtual void setData(Data *d) = 0;
    virtual double execute() = 0; // should return the percentage of coupled points so the pipeline can tell if the matching went well.

    motion3D *findBestMotion(float thrs, Base *BA, Base *BB, double &minRes, float &percPairedPoints,
                                                 motion3D *bestMotion, float percOfPoints);

    Point * findCorrespondence(Point *p, ElementSet *es);   // Method to find a correspondence point.
    Point * findWidePoint(Point *p, ElementSet *es);        // Finds a wide partner of a given point.
    bool checkDistances(float thrs, Base *A, Base *B);                  // Check distances between bases.
    bool checkDescriptors(float thrs, Point *ai,
            Point *aj, Point *ak,
            Point *bi, Point *bj, Point *bk);               // Check descriptor distance.
    bool checkMotion(Base *A, Base *B);                     // Check motion with few points.
    bool checkNormals(Base *A, Base *B);                    // Check normal diferecnes between points.

    static double calcArea(Point *a, Point *b, Point *c);


};

#endif //ISEARCHINGSTRATEGY_H
