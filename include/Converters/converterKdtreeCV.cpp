//
// Created by ferran on 9/06/16.
//

#include "converterKdtreeCV.h"


converterKdtreeCV::converterKdtreeCV() {

}

converterKdtreeCV::~converterKdtreeCV() {

}

cv::Mat * converterKdtreeCV::convertArray(vector<Point *> *P) {

    cv::Mat *points = new cv::Mat(P->size(), 3, CV_32F);

    for(int i=0; i<P->size(); i++){


        points->at<float>(0, i) = P->at(i)->getX();
        points->at<float>(1, i) = P->at(i)->getY();
        points->at<float>(2, i) = P->at(i)->getZ();


//        vector<float> p;
//        p.push_back(P->at(i)->getX());
//        p.push_back(P->at(i)->getY());
//        p.push_back(P->at(i)->getZ());
//
//        cv::Mat point(p, true);
//        point = point.t();
//
//        points->push_back(point);
    }

    return points;
}

vector<float> converterKdtreeCV::convertPoint(Point *p) {


    vector<float> point;

    point.push_back(p->getX());
    point.push_back(p->getY());
    point.push_back(p->getZ());

    return point;
}