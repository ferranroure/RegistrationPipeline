//
// Created by ferran on 4/03/16.
//
#include "converterGridTree.h"


converterGridTree::converterGridTree() {

}

converterGridTree::~converterGridTree() {

}

vector<myPoint *> converterGridTree::convertArray(vector<Point *> *P) {

    vector<myPoint *> res;

    for (int i = 0; i < P->size(); ++i) {

        myPoint *e = new myPoint(P->at(i)->getX(), P->at(i)->getY(), P->at(i)->getZ());

        e->setIndex(P->at(i)->getIndex());

        res.push_back(e);
    }

    return res;
}

myPoint *converterGridTree::convertPoint(Point *p) {

    return new myPoint(p->getX(), p->getY(),p->getZ(), p->getIndex());
}

