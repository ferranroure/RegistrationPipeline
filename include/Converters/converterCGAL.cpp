//
// Created by Ferran Roure on 26/10/2015.
//

#include "converterCGAL.h"

converterCGAL::converterCGAL() {

}

converterCGAL::~converterCGAL() {

}


list<PointVectorPair> converterCGAL::points2CGAL_list(vector<Point *> *cloud) {

    list<PointVectorPair> pts;

    for (int i = 0; i < cloud->size(); ++i) {

        Point_3 p(cloud->at(i)->getX(), cloud->at(i)->getY(), cloud->at(i)->getZ());

        PointVectorPair pvp;
        pvp.first = p;

        pts.push_back(pvp);
    }

    return pts;
}

vector<vector3D> *converterCGAL::CGAL2normals(list<PointVectorPair> points) {

    vector<vector3D> * normals = new vector<vector3D>;

    for(list<PointVectorPair>::iterator it = points.begin(); it!=points.end(); ++it){


        vector3D norm(it->second.x(), it->second.y(), it->second.z());

        normals->push_back(norm);
    }

    return normals;
}
