//
// Created by ferran on 19/01/16.
//

#include "myCompressedOctree.h"


myCompressedOctree::myCompressedOctree() {

    cOctree = NULL;
    ads = NULL;
}

myCompressedOctree::myCompressedOctree(vector<Point *> *P, float _diag) {

    diagonal = _diag;
    ads = new AdapterDataStruct();
    vector<Element *> points = ads->convertArray(P);
    cOctree = new CompressedOctree(points, 0);
}

myCompressedOctree::~myCompressedOctree() {

    delete cOctree;
    delete ads;
}

returnData myCompressedOctree::calcOneNN(Point *queryPoint, float errEps) {

    point3D p(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());
    Element *aux = new Element(p, 0);

    list<Element> vnn;
    vnn = cOctree->weightedNeighbors(aux, errEps);

    Element *nn = new Element();

    float bestDist = FLT_MAX;
    for(list<Element>::iterator it = vnn.begin(); it!=vnn.end(); ++it ){

        float dist = p.dist(it->getPoint());
        if(dist<bestDist && it->getPoint()!=p){
            bestDist = dist;
            nn->setPoint(it->getPoint());
        }
    }

//    Element * nn = trihash->nearestNeighbor(p);

    returnData rd;
    if(vnn.empty()){
        rd.index = -1;
        rd.sqrDist = FLT_MAX;
    }
    else {
        rd.index = nn->getIndex();
        Point *pnn = new Point(nn->getPoint().getX(), nn->getPoint().getY(), nn->getPoint().getZ());
        float dist = queryPoint->dist(pnn);
        rd.sqrDist = dist * dist;
        delete pnn;
    }

    return rd;

}

returnData myCompressedOctree::calcOwnNN(Point *queryPoint) {

    point3D p(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());
    Element *aux = new Element(p, 0);

    list<Element> vnn;
    int factor = 1;
    while(vnn.empty()) {
        vnn = cOctree->weightedNeighbors(aux, diagonal * 0.01 * factor);
        ++factor;
    }
    Element *nn = new Element();

    float bestDist = FLT_MAX;
    for(list<Element>::iterator it = vnn.begin(); it!=vnn.end(); ++it ){

        float dist = p.dist(it->getPoint());
        if(dist<bestDist && it->getPoint()!=p){
            bestDist = dist;
            nn->setPoint(it->getPoint());
        }
    }

//    Element * nn = trihash->nearestNeighbor(p);

    returnData rd;
    if(vnn.empty()){
        rd.index = -1;
        rd.sqrDist = FLT_MAX;
    }
    else {
        rd.index = nn->getIndex();
        Point *pnn = new Point(nn->getPoint().getX(), nn->getPoint().getY(), nn->getPoint().getZ());
        float dist = queryPoint->dist(pnn);
        rd.sqrDist = dist * dist;
        delete pnn;
    }

    return rd;
}

vector<returnData> myCompressedOctree::calcNneigh(Point *queryPoint, int nNeigh) {
    return std::vector<returnData>();
}
