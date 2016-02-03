//
// Created by ferran on 04/05/15.
//

#include "converter4PCS.h"

converter4PCS::converter4PCS() {

}

converter4PCS::~converter4PCS() {

}

vector<Point3D> *converter4PCS::points24PCS(vector<Point *> *cloud, bool withNormals, bool withColor) {

    vector<Point3D> * pts = new vector<Point3D>();

    for (int i = 0; i < cloud->size(); ++i) {

        Point *myP = cloud->at(i);

        Point3D p(myP->getX(), myP->getY(), myP->getZ());

        if(withNormals){
            p.n1 = myP->getNormal()->getX();
            p.n2 = myP->getNormal()->getY();
            p.n2 = myP->getNormal()->getZ();
        }
        if(withColor){
            p.r = myP->getRed();
            p.g = myP->getGreen();
            p.b = myP->getBlue();
        }

        pts->push_back(p);
    }

    return pts;
}

motion3D *converter4PCS::mat2motion(double mat[4][4]) {

    return new motion3D(mat);
}
