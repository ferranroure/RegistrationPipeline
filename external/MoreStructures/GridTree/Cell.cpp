//
// Created by ferran on 4/03/16.
//

#include "Cell.h"

Cell::Cell() {

    kdTree = NULL;

}

Cell::~Cell() {

    if(kdTree!=NULL){
        delete kdTree;
    }

}

void Cell::addPoint(myPoint *p) {

    points.push_back(p);
}

int Cell::get_nPoints() {

    return points.size();
}

myPoint *Cell::getPoint(int pos) {

    return points.at(pos);
}
