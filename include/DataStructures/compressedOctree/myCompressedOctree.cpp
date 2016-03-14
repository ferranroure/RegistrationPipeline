//
// Created by ferran on 19/01/16.
//

#include "myCompressedOctree.h"


myCompressedOctree::myCompressedOctree() {

    cOctree = NULL;
    cyg = NULL;
}

myCompressedOctree::myCompressedOctree(vector<Point *> *P, float _diag) {

    diagonal = _diag;
    cyg = new converterYago();
    vector<Element *> points = cyg->convertArray(P);
    cOctree = new CompressedOctree(points, 0);
}

myCompressedOctree::~myCompressedOctree() {

    delete cOctree;
    delete cyg;
}

returnData myCompressedOctree::calcOneNN(Point *queryPoint, float errEps) {

    point3D p(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());
    Element *aux = new Element(p, 0);

//    CompressedONode *t = cOctree->getArrel();
//    CompressedONode nodeAux = CompressedONode(t->getAncoratge(),t->getMida(),t->getNivell() );
//    vector<bool> fillsTocats = cOctree->descomprimeix(t,&nodeAux);

    list<Element*> *vnn  = cOctree->weightedNeighbors(aux, errEps);

//    cOctree->recomprimeix(fillsTocats,&nodeAux);
//    delete t;


    Element *nn = new Element();

    if(vnn!=NULL) {

        float bestDist = FLT_MAX;
        for (list<Element *>::iterator it = vnn->begin(); it != vnn->end(); ++it) {

            float dist = p.dist((*it)->getPoint());
            if (dist < bestDist && (*it)->getPoint() != p && dist <= errEps) {
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

returnData myCompressedOctree::calcOwnNN(Point *queryPoint) {

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

vector<returnData> myCompressedOctree::calcNneigh(Point *queryPoint, int nNeigh) {
    return std::vector<returnData>();
}

void myCompressedOctree::printStats() {

    cout << cOctree->getNivellActual() << endl;
}
