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

    //for (int i = 0; i < points.size(); ++i) {
    //    delete points[i];
   // }


    points.clear();
}

myCompressedOctree::~myCompressedOctree() {

  if(cOctree!=NULL)  delete cOctree;
   if(cyg!=NULL) delete cyg;
}

returnData myCompressedOctree::calcOneNN(Point *queryPoint, float errEps) {

//    returnData rd;
//    rd.index = -1;
//    rd.sqrDist = FLT_MAX;
 //   return rd;

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

    if( vnn != NULL ) {

        vnn->clear();
        delete vnn;
    }
    delete aux;
    delete nn;

    return rd;

}

returnData myCompressedOctree::calcOwnNN(Point *queryPoint) {

    //returnData rd;
     //   rd.index = -1;
     //   rd.sqrDist = FLT_MAX;
    //return rd;

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

    if( vnn != NULL ) {

        vnn->clear();
        delete vnn;
    }
    delete aux;
    delete nn;

   // cout<<"mycoimpressed octree ownNN retornant*********************************************************************************** "<<rd.index<<" "<<rd.sqrDist<<endl;

    return rd;

}

vector<returnData> myCompressedOctree::calcNneigh(Point *queryPoint, int nNeigh) {
    return std::vector<returnData>();
}
