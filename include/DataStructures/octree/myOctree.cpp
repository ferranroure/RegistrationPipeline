//
// Created by ferran on 15/01/16.
//

#include "myOctree.h"

myOctree::myOctree() {

    octree = NULL;
}

myOctree::myOctree(vector<Point *> *P) {

    float resolution = 128.0f;
    octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = apcl.points2PCL(P);

    octree->setInputCloud (cloud);
    octree->addPointsFromInputCloud ();

}

myOctree::~myOctree() {

    delete octree;
}

returnData myOctree::calcOneNN(Point *queryPoint, float errEps) {

    vector<returnData> vrd = calcNneigh(queryPoint, 1);

    return vrd.at(0);
}

returnData myOctree::calcOwnNN(Point *queryPoint) {

    vector<returnData> vrd = calcNneigh(queryPoint, 2);

    return vrd.at(1);
}

vector<returnData> myOctree::calcNneigh(Point *queryPoint, int nNeigh) {


    vector<int> pointIdxNKNSearch;
    vector<float> pointNKNSquaredDistance;
    pcl::PointXYZ searchPoint(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());

    octree->nearestKSearch (searchPoint, nNeigh, pointIdxNKNSearch, pointNKNSquaredDistance);


    vector<returnData> vrd;

    for(int i=0; i<pointIdxNKNSearch.size(); ++i){

        returnData rd;
        rd.sqrDist = pointNKNSquaredDistance.at(i);
        rd.index = pointIdxNKNSearch.at(i);
        vrd.push_back(rd);
    }

    return vrd;
}
