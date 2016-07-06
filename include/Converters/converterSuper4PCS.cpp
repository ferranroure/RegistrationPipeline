//
// Created by ferran on 6/07/16.
//

#include "converterSuper4PCS.h"

converterSuper4PCS::converterSuper4PCS() {

}

converterSuper4PCS::~converterSuper4PCS() {

}

vector<match_4pcs::Point3D> & converterSuper4PCS::convertArray(vector<Point*> *cloud, bool withNormals, bool withColor){

    vector<match_4pcs::Point3D> *pts = new vector<match_4pcs::Point3D>();

    for (int i = 0; i < cloud->size(); ++i) {

        Point *myP = cloud->at(i);

        match_4pcs::Point3D p(myP->getX(), myP->getY(), myP->getZ());

        if(withNormals && myP->getNormal()!=NULL){
            match_4pcs::Point3D normal(myP->getNormal()->getX(), myP->getNormal()->getY(),myP->getNormal()->getZ());
            p.set_normal(normal);
        }
        if(withColor){
            cv::Vec3f color(myP->getRed(), myP->getGreen(), myP->getBlue());
            p.set_rgb(color);
        }

        pts->push_back(p);
    }

    return *pts;
}

motion3D * converterSuper4PCS::convertMatrix(cv::Mat mat){

    double aux[4][4];

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            aux[i][j] = mat.at<double>(i,j);
        }
    }

    return new motion3D(aux);
}