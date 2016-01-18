//
// Created by ferran on 23/07/15.
//

#ifndef NODE_H
#define NODE_H

#define INSIDE 1
#define OUTSIDE 0
#define INTERSECT 2
#define CONTAINS 3

#include <iostream>
#include <vector>
#include "../../point.h"
#include "../IDataStructure.h"


class Node {

public:
    float xmin, xmax, ymin, ymax, zmin, zmax;
    vector<Point> vertexs;
    bool isLeaf;
    vector<Node*> children;
    vector<Point*> points;
    int level;

    Node();
    Node(float XMIN, float XMAX, float YMIN, float YMAX, float ZMIN, float ZMAX, int LEVEL);
    ~Node();
    void addChild(Node* child);
    void addPoint(Point *p);
    bool isInside(Point* p);
    bool intersect(float A, float B, float C, float D);
    vector<Point *> findCoplanarPointsRecursive(float A, float B, float C, float D, Point *p, float dist, int &calls,
                                                int &posCalls);
    vector<pair<Point*, Point*> > findPairsRecursive(Point *p, float dist, float thrs);
    bool checkDistance(float maxRadi, float minRadi, Point *p);
    int checkIntersection(Node &box);

    void print();
    void printInfo();
    void createPly(vector<Point> &vp);

    returnData * calcOneNN(Point *queryPoint);
    vector<returnData> * calcNneigh(Point *queryPoint, int nNeigh);

    void test();
};


#endif //NODE_H
