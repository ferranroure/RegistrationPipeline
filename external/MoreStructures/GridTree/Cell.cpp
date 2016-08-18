//
// Created by ferran on 4/03/16.
//

#include "Cell.h"


Cell::Cell() {

    dataStructure = NULL;
    points = new vector<Point*>();

}

Cell::~Cell() {


    if(dataStructure!=NULL){
        delete dataStructure;
    }

    delete points; // only deleting vector, not de pointers inside.
}

void Cell::kdtreezation(string dsType, int thsPoints, float diagonal) {


    if(points->size() > thsPoints){


        if(dataStructure!=NULL) delete dataStructure;

        if(dsType=="kdtree"){
            dataStructure = new myKdtree(points);
        }
        else if(dsType=="compressedOctree"){
            dataStructure = new myCompressedOctree(points, diagonal);
        }
        else if(dsType=="trihash"){
            dataStructure = new myTriHash(points, diagonal);
        }
        else if(dsType=="S4PCSkdtree"){
            dataStructure = new myS4PCSkdtree(points, diagonal);
        }
        else{
            cerr << "I can't understand your dataStructureure!" << endl;
            exit(EXIT_FAILURE);
        }

    }
}


bool Cell::isKdtreezed() {

    return dataStructure != NULL;
}

void Cell::addPoint(Point *p) {

    points->push_back(p);
}

int Cell::get_nPoints() {

    return points->size();
}

bool Cell::empty(){

    return points->empty();
}


IDataStructure * Cell::getDataStructure() {

    return dataStructure;
}

Point *Cell::getPoint(int pos) {

    return points->at(pos);
}

