//
// Created by ferran on 18/01/16.
//

#include "converterYago.h"

// set index in convertArray()


converterYago::converterYago(){

}

converterYago::~converterYago() {

}

vector<Element *> converterYago::convertArray(vector<Point *> *P) {

    vector<Element *> res;

    for (int i = 0; i < P->size(); ++i) {

        Element *e = new Element();
        e->setPoint(point3D(P->at(i)->getX(), P->at(i)->getY(), P->at(i)->getZ()));
        e->setIndex(i);


        res.push_back(e);
        //if(i==0)cout<<"IS TIHS THE SAME?"<<*res[0]<<endl;

        //delete e;
    }
//cout<<"**************************************************************************************IS TIHS THE SAME?"<<*res[0]<<endl;
    return res;
}

point3D * converterYago::convertPoint(Point *p){

  return new point3D(p->getX(), p->getY(),p->getZ());
}
