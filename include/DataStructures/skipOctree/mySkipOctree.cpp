//
// Created by yago on 16/02/26.
//

#include "mySkipOctree.h"


mySkipOctree::mySkipOctree() {

    cOctree = NULL;
    cyg = NULL;
}

mySkipOctree::mySkipOctree(vector<Point *> *P, float _diag) {

diagonal = _diag;
cyg = new converterYago();
vector<Element *> points = cyg->convertArray(P);
cOctree = new SkipOctree(points, 0);
}

mySkipOctree::~mySkipOctree() {

    delete cOctree;
    delete cyg;
}

returnData mySkipOctree::calcOneNN(Point *queryPoint, float errEps) {

    point3D p(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());
    Element *aux = new Element(p, 0);

    list<Element*> *vnn  = cOctree->weightedNeighbors(aux, errEps);

    Element *nn = new Element();

    if(vnn!=NULL) {

        float bestDist = FLT_MAX;
        for (list<Element *>::iterator it = vnn->begin(); it != vnn->end(); ++it) {

            float dist = p.dist((*it)->getPoint());
            if (dist < bestDist && (*it)->getPoint() != p) {
                bestDist = dist;
                nn->setPoint((*it)->getPoint());
            }
        }
    }

    returnData rd;
    if( vnn == NULL || vnn->empty()){
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

    delete vnn;
    delete aux;
    delete nn;

    return rd;

}

returnData mySkipOctree::calcOwnNN(Point *queryPoint) {

    point3D p(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());
    Element *aux = new Element(p, 0);

    int factor = 1;
    list<Element*> *vnn = cOctree->weightedNeighbors(aux, diagonal * 0.01 * factor);
    ++factor;
    while(vnn->empty()) {
        vnn = cOctree->weightedNeighbors(aux, diagonal * 0.01 * factor);
        ++factor;
    }

    Element *nn = new Element();

    float bestDist = FLT_MAX;
    for(list<Element*>::iterator it = vnn->begin(); it!=vnn->end(); ++it ){

        float dist = p.dist((*it)->getPoint());
        if(dist<bestDist && (*it)->getPoint()!=p){
            bestDist = dist;
            nn->setPoint((*it)->getPoint());
        }
    }

//    Element * nn = trihash->nearestNeighbor(p);

    returnData rd;
    if(vnn->empty()){
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

    delete vnn;
    delete aux;
    delete nn;

    return rd;
}

vector<returnData> mySkipOctree::calcNneigh(Point *queryPoint, int nNeigh) {
    return std::vector<returnData>();
}
