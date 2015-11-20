//
// Created by ferran on 23/07/15.
//

#include "Node.h"

Node::Node() {

    level = 0;
    isLeaf = false;
}

Node::Node(float XMIN, float XMAX, float YMIN, float YMAX, float ZMIN, float ZMAX, int LEVEL) {

    xmin = XMIN;
    xmax = XMAX;
    ymin = YMIN;
    ymax = YMAX;
    zmin = ZMIN;
    zmax = ZMAX;
    level = LEVEL;
    isLeaf=false;

    vertexs.push_back(Point(xmin, ymin, zmin));
    vertexs.push_back(Point(xmin, ymax, zmin));
    vertexs.push_back(Point(xmax, ymax, zmin));
    vertexs.push_back(Point(xmax, ymin, zmin));
    vertexs.push_back(Point(xmin, ymin, zmax));
    vertexs.push_back(Point(xmin, ymax, zmax));
    vertexs.push_back(Point(xmax, ymax, zmax));
    vertexs.push_back(Point(xmax, ymin, zmax));
}

Node::~Node() {

    if (!isLeaf) {
        for (int i = 0; i < children.size(); ++i) {

            delete children.at(i);
        }
    }
}

void Node::addChild(Node *child) {

    if(children.size() < 8) {
        children.push_back(child);
    }
    else{
        cerr << "This node is full!!" << endl;
    }
}


void Node::addPoint(Point *p) {

    points.push_back(p);
}

bool Node::isInside(Point *p) {

    if(p->getX() >= xmin && p->getX() <= xmax &&
       p->getY() >= ymin && p->getY() <= ymax &&
       p->getZ() >= zmin && p->getZ() <= zmax) {

        return true;
    }
    else{
        return false;
    }
}

bool Node::intersect(float A, float B, float C, float D){

    // Check if the plane ABCD intersects with this node.
    // We have to check if each edge intersects with the plane.
    // Check if one point of the edge is on the positive side and the other is on the negative side.

//    srand(time(NULL));
//    if(rand()%2 == 0) return true;
//    else return false;


    bool found = false;


    // checking both squares (zmin & zmax)
    for (int i = 0; i <= 4; i=i+4) {

        for (int j = 0; j < 4; ++j) {

            Point p = vertexs.at(j+i);
            Point q;
            if (j < 3) q = vertexs.at((j+i) + 1);
            else q = vertexs.at(0+i);

            float res1 = A*p.getX() + B*p.getY() + C*p.getZ() - D;
            float res2 = A*q.getX() + B*q.getY() + C*q.getZ() - D;

            if(res1*res2 <= 0.0f) return true;

        }
    }

    // checking edges between both z squares.
    for (int k = 0; k < 4; ++k) {
        Point p = vertexs.at(k);
        Point q = vertexs.at(k+4);

        float res1 = A*p.getX() + B*p.getY() + C*p.getZ() - D;
        float res2 = A*q.getX() + B*q.getY() + C*q.getZ() - D;

        if(res1*res2 <= 0.0f) return true;
    }

    return false;

}


vector<Point *> Node::findCoplanarPointsRecursive(float A, float B, float C, float D, Point *p, float dist, int &calls,
                                            int &posCalls) {

    vector<Point*> res;

    calls++;
    float thrs = 0.00001;

    // If distance == -1 means that this method is used to find a initial Base, that means
    // that there is no distance restriction. We only want all points lying on the orthogonal plane.
    if(checkDistance(dist+thrs, dist-thrs, p) || dist == -1){
        if(intersect(A,B,C,D)) {

            posCalls++;

            if (isLeaf) {
                res = points;
            }
            else {
                for (int i = 0; i < children.size(); ++i) {

                    vector<Point *> temp = children.at(i)->findCoplanarPointsRecursive(A, B, C, D, p, dist, calls,
                                                                                       posCalls);
                    res.insert(res.end(), temp.begin(), temp.end());
                }
            }
        }
    }

    return res;
}

/*
 * Metod checkDistance
 *
 * We want to find all points lying at distance D. This distance D have some error (thrs). For thes reason, we find
 * points lying inside a crown between D+thrs and D-thrs. We simplify the intersection between Nodes and spheres using
 * cubes.
 * This method checks the interesection of the actual node with two different cubes (represented by a Node object)
 * outCube and inCube are the simplification of the sphere of radius D+thrs and D-thrs. We check the types of interesction
 * between the actual node and the "sphere-nodes".
 * Cases of right intersection:
 * Node INSIDE outCube & Node OUTSIDE inCube
 * Node INSIDE outCube & Node INTERSECT inCube
 * Node INTERSECT outCube & Node OUTSIDE inCube
 * Node INTERSECT outCube & Node INTERSECT inCube
 * Node CONTAIN both cubes
 */
bool Node::checkDistance(float maxRadi, float minRadi, Point *p) {


    Node outCube(   p->getX() - maxRadi,
                    p->getX() + maxRadi,
                    p->getY() - maxRadi,
                    p->getY() + maxRadi,
                    p->getZ() - maxRadi,
                    p->getZ() + maxRadi, 0);

    Node inCube(    p->getX() - minRadi,
                    p->getX() + minRadi,
                    p->getY() - minRadi,
                    p->getY() + minRadi,
                    p->getZ() - minRadi,
                    p->getZ() + minRadi, 0);

//    cout << "Node xmin: " << xmin << " outCube xmin: " << outCube.xmin << endl;

    int resOut = this->checkIntersection(outCube);
    int resIn = this->checkIntersection(inCube);

    if (    (resOut == INSIDE && resIn == OUTSIDE)      ||
            (resOut == INSIDE && resIn == INTERSECT)    ||
            (resOut == INTERSECT && resIn == OUTSIDE)   ||
            (resOut == INTERSECT && resIn == INTERSECT) ||
            (resOut == CONTAINS  && resIn == CONTAINS)    ){

        return true;
    }
    else{
        return false;
    }

}
/*
 *  Node n is the box that represents the sphere of radius r, centered at point p.
 */
int Node::checkIntersection(Node &box) {

    int stateX = -1;
    int stateY = -1;
    int stateZ = -1;



    // Find the state for each axis.
    if(box.xmin <= xmin && xmax <= box.xmax) stateX = INSIDE;
    if(box.ymin <= ymin && ymax <= box.ymax) stateY = INSIDE;
    if(box.zmin <= zmin && zmax <= box.zmax) stateZ = INSIDE;

    if(xmin < box.xmin && box.xmax < xmax) stateX = CONTAINS;
    if(ymin < box.ymin && box.ymax < ymax) stateY = CONTAINS;
    if(zmin < box.zmin && box.zmax < zmax) stateZ = CONTAINS;

    if ((xmax < box.xmin) || (box.xmax < xmin)) stateX = OUTSIDE;
    if ((ymax < box.ymin) || (box.ymax < ymin)) stateY = OUTSIDE;
    if ((zmax < box.zmin) || (box.zmax < zmin)) stateZ = OUTSIDE;

    if ((xmin < box.xmin && box.xmin < xmax && xmax < box.xmax) || (box.xmin < xmin && xmin < box.xmax && box.xmax < xmax)) stateX = INTERSECT;
    if ((ymin < box.ymin && box.ymin < ymax && ymax < box.ymax) || (box.ymin < ymin && ymin < box.ymax && box.ymax < ymax)) stateY = INTERSECT;
    if ((zmin < box.zmin && box.zmin < zmax && zmax < box.zmax) || (box.zmin < zmin && zmin < box.zmax && box.zmax < zmax)) stateZ = INTERSECT;


    // MERGING STATES
    if(stateX==OUTSIDE || stateY==OUTSIDE || stateZ==OUTSIDE) return OUTSIDE;

    // a partir d'aquí és impossible que un eix estigui fora.
    if(stateX==INTERSECT || stateY==INTERSECT || stateZ==INTERSECT) return INTERSECT;

    // A partir d'aquí només pot quedar INSIDE i CONTAINS
    // If one of those are different from each other the node must INTERSECT.
    if(stateX != stateY || stateY != stateZ || stateZ != stateX) return INTERSECT;

    if(stateX==INSIDE && stateY==INSIDE && stateZ==INSIDE) return INSIDE;

    if(stateX==CONTAINS && stateY==CONTAINS && stateZ==CONTAINS) return CONTAINS;

    printInfo();
    cout << stateX << " " << stateY << " " << stateZ << endl;
    cout << "BOOOOOOX. " << endl;
    box.printInfo();

    cerr << "CheckIntersection::Something wrong happens. I can't compute the intersection!!" << endl;
    cerr << "I'm on level: " << level << endl;
    exit(EXIT_FAILURE);

    return -1;
}


vector<pair<Point*, Point*> > Node::findPairsRecursive(Point *p, float dist, float thrs) {

    vector<pair<Point*, Point*> > res;

    if (isLeaf) {
//        cout << "socfulla" << endl;
        for (int j = 0; j < points.size(); ++j) {

            if (abs(p->dist(points.at(j)) - dist) <= thrs) {

                pair<Point *, Point *> two(p, points.at(j));
                res.push_back(two);

            }
        }
    }
    else {
        if (checkDistance(dist + thrs, dist - thrs, p)) {
            for (int i = 0; i < children.size(); ++i) {

                vector<pair<Point*, Point*> > temp = children.at(i)->findPairsRecursive(p, dist, thrs);
                res.insert(res.end(), temp.begin(), temp.end());
            }
        }
    }

    return res;
}

void Node::print() {

    string espai = " ";

    for (int j = 0; j < level; ++j) {

        cout << espai;
    }

//    if(points.size()!=0) {
        cout << points.size() << " p." << endl;
//    }

    if( !isLeaf ) {

        for (int i = 0; i < children.size(); ++i) {

            children.at(i)->print();
        }
    }
}


void Node::createPly(vector<Point> &vp) {

    vp.push_back(Point(xmin, ymin, zmin));
    vp.push_back(Point(xmin, ymin, zmax));
    vp.push_back(Point(xmin, ymax, zmin));
    vp.push_back(Point(xmin, ymax, zmax));
    vp.push_back(Point(xmax, ymin, zmin));
    vp.push_back(Point(xmax, ymin, zmax));
    vp.push_back(Point(xmax, ymax, zmin));
    vp.push_back(Point(xmax, ymax, zmax));

    for (int i = 0; i < children.size(); ++i) {

        children.at(i)->createPly(vp);
    }
}

void Node::test(){

    Node box(0, 10, 0, 10, 0, 10, 0); // el box

    Node no0(-2, 3, 2, 3, 2, 3, 0); // intersecta 1 cara
    Node no1(2, 3, 2, 3, 20, 30, 0); // un eix està fora
    Node no2(2,3,2,3,2,3,0); // es a dins
    Node no3(9,12, 9, 12, 9, 12, 0); // intersecta a una cantonada
    Node no4(9,12, 9, 12, 2, 7, 0); // intersecta una aresta
    Node no5 (15, 20, 15, 20, 15, 20, 0); // està fora.
    Node no6(-5, 15, -5, 15, -5, 15, 0); // conté el node.

    cout << "ha de donar 2: " << no0.checkIntersection(box) << endl;
    cout << "ha de donar 0: " << no1.checkIntersection(box) << endl;
    cout << "ha de donar 1: " << no2.checkIntersection(box) << endl;
    cout << "ha de donar 2: " << no3.checkIntersection(box) << endl;
    cout << "ha de donar 2: " << no4.checkIntersection(box) << endl;
    cout << "ha de donar 0: " << no5.checkIntersection(box) << endl;
    cout << "ha de donar 3: " << no6.checkIntersection(box) << endl;

    exit(0);
}

void Node::printInfo() {

    cout << "NODE INFORMATION: " << endl;
    cout << "   Level: " << level << endl;
    cout << "   xmin: " << xmin << " xmax: " << xmax << endl;
    cout << "   ymin: " << ymin << " ymax: " << ymax << endl;
    cout << "   zmin: " << zmin << " zmax: " << zmax << endl;
    cout << "   nPoints: " << points.size() << endl << endl;
}
