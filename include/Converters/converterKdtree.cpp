#include "converterKdtree.h"


converterKdtree::converterKdtree(){ }

converterKdtree::~converterKdtree() { }

/* CONVERT ARRAY FOR KDTREE -----------------------------------------
 *
 *  This method is a bridge between our point storing format and
 *  ANNkdtree point storing format. Given a vector<Point>, it transforms
 *  the data to ANNpointArray with ANNpoints.
 */
ANNpointArray converterKdtree::convertArray(vector<Point *> *P){

    ANNpointArray pa;
    pa = annAllocPts(P->size(), DIMENSIONS);
    for (int i = 0; i < P->size(); ++i) {

        pa[i][0] = P->at(i)->getX();
        pa[i][1] = P->at(i)->getY();
        pa[i][2] = P->at(i)->getZ();
    }

    return pa;
}


/* CONVERT POINT FOR KDTREE -----------------------------------------
 *
 *  This method tranform a given Point to an ANNpoint from ANNkdtree class
 */
ANNpoint converterKdtree::convertPoint(Point *p)
{
    ANNpoint annP;
    annP = annAllocPt(3);
    annP[0] = p->getX();
    annP[1] = p->getY();
    annP[2] = p->getZ();

    return annP;
}
