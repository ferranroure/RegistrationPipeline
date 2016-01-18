//
// Created by ferran on 18/01/16.
//

#include "AdapterDataStruct.h"

// set index in convertArray()


AdapterDataStruct::AdapterDataStruct(){

}

AdapterDataStruct::~AdapterDataStruct() {

}

vector<Element *> AdapterDataStruct::convertArray(vector<Point *> *P) {

    vector<Element *> res;

    for (int i = 0; i < P->size(); ++i) {

        Element *e = new Element();
        e->setPoint(point3D(P->at(i)->getX(), P->at(i)->getY(), P->at(i)->getZ()));
        e->setIndex(i);

        res.push_back(e);
    }

    return res;
}
