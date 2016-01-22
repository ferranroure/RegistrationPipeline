//
// Created by ferran on 23/07/15.
//

#ifndef OCTREE_H
#define OCTREE_H

#include <iostream>
#include <vector>
#include "Node.h"
#include "../../plyio.h"
#include "../IDataStructure.h"

class Octree : public IDataStructure{

public:
    Node * root;
    int maxLevel;

public:

    Octree();
    Octree(vector<Point *> *points, int MAXLEVEL, float xmin, float xmax, float ymin, float ymax, float zmin,
                       float zmax);
    ~Octree();

    vector<Point *> findCoplanarPoints(vector3D norm, float d, Point *p, float dist);
    vector<pair<Point*, Point*> > findPairs(Point *p, float dist, float thrs);


    void createOctree(Node *father, int actLevel);
    void print();
    void createPly();

    returnData calcOneNN(Point *queryPoint, float errEps);
    returnData calcOwnNN(Point *queryPoint);
    vector<returnData> calcNneigh(Point *queryPoint, int nNeigh);
};


#endif //OCTREE_H
