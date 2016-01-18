//
// Created by ferran on 23/07/15.
//

#include "Node.h"
#include "Octree.h"

Octree::Octree() {

}

Octree::Octree(vector<Point *> *points, int MAXLEVEL, float xmin, float xmax, float ymin, float ymax, float zmin,
                   float zmax) {

    maxLevel = MAXLEVEL;

    // build the root node (level 0);

    root = new Node(xmin, xmax, ymin, ymax, zmin, zmax, 0);
    root->isLeaf = false;
    root->level = 0;

    for (int i = 0; i < points->size(); ++i) {
        root->addPoint(points->at(i));
    }

    createOctree(root, root->level + 1);
}

Octree::~Octree() {

}

vector<Point *> Octree::findCoplanarPoints(vector3D norm, float d, Point *p, float dist) {
    /*
    COSES A FER:

    Hem de mofificar el mètode de Node perquè vagi cridant recursivament els fills. S'ha d'anar baixant fins
    a les fulles i allà reportar els punts dins un vector. Cada pare anirà fusionant els vectors fins a reportar el
    vector final a aquest mètode.

*/

    int calls = 0;
    int posCalls = 0;

    vector<Point*> cCand = root->findCoplanarPointsRecursive(norm.getX(), norm.getY(), norm.getZ(), d, p, dist,
                                                             calls, posCalls);

//    cout << "calls: " << calls << endl;
//    cout << "positive calls: " << posCalls << endl;

    return cCand;
}


vector<pair<Point*, Point*> > Octree::findPairs(Point *p, float dist, float thrs) {

    return root->findPairsRecursive(p, dist, thrs);

}

void Octree::createOctree(Node *father, int actLevel) {


    // FALTA COMPROVAR QUÈ PASSA SI ENCARA QUEDEN LEVELS PERO NO QUEDEN PUNTS PER POSAR.

    if(father->points.size() > 5){
    //if(actLevel <= maxLevel){

        float xmin = father->xmin;
        float xmid = (father->xmax + father->xmin)/2;
        float xmax = father->xmax;
        float ymin = father->ymin;
        float ymid = (father->ymax + father->ymin)/2;
        float ymax = father->ymax;
        float zmin = father->zmin;
        float zmid = (father->zmax + father->zmin)/2;
        float zmax = father->zmax;

        father->addChild(new Node(xmin, xmid, ymin, ymid, zmin, zmid, actLevel));
        father->addChild(new Node(xmid, xmax, ymin, ymid, zmin, zmid, actLevel));
        father->addChild(new Node(xmin, xmid, ymid, ymax, zmin, zmid, actLevel));
        father->addChild(new Node(xmid, xmax, ymid, ymax, zmin, zmid, actLevel));
        father->addChild(new Node(xmin, xmid, ymin, ymid, zmid, zmax, actLevel));
        father->addChild(new Node(xmid, xmax, ymin, ymid, zmid, zmax, actLevel));
        father->addChild(new Node(xmin, xmid, ymid, ymax, zmid, zmax, actLevel));
        father->addChild(new Node(xmid, xmax, ymid, ymax, zmid, zmax, actLevel));


        for (int i = 0; i < father->points.size(); ++i) {

            for (int j = 0; j < father->children.size(); ++j) {

                Node *child = father->children.at(j);
                if(child->isInside(father->points.at(i))) {

                    child->addPoint(father->points.at(i));
                    break;
                }
            }
        }

        // Recursive execution for each children.
        for (int j = 0; j < father->children.size(); ++j) {

            createOctree(father->children.at(j), actLevel + 1);
        }

/*
        if(actLevel < maxLevel){
            for (int j = 0; j < father->children.size(); ++j) {

                Node *child = father->children.at(j);
                if(child->points.empty()){
                    child->isLeaf = true;
                }
                else {
                    createOctree(child, actLevel + 1);
                }
            }
        }
        else {// actLevel == maxLevel

            for (int j = 0; j < father->children.size(); ++j) {

                Node *child = father->children.at(j);
                child->isLeaf = true;
            }
        }
*/
    }
    else{
        father->isLeaf = true;
        //cout << father->points.size() << endl;
        //cout << father->level << endl;
    }
}


void Octree::print() {

    root->print();
}

void Octree::createPly(){

    vector<Point> vp;

    root->createPly(vp);

    PlyIO ply;

    ply.writeFile("../../models/ResidueTests/octree.ply", &vp);
}

returnData Octree::calcOneNN(Point *queryPoint) {

    returnData * rd = root->calcOneNN(queryPoint);

    returnData rrd;

    if(rd != NULL) {
        rrd.index = rd->index;
        rrd.sqrDist = rd->sqrDist;
    }
    else{
        rrd.index = -1;
        rrd.sqrDist = FLT_MAX;
    }
    return rrd;
}

returnData Octree::calcOwnNN(Point *queryPoint) {
    return returnData();
}

vector<returnData> Octree::calcNneigh(Point *queryPoint, int nNeigh) {

    vector<returnData> *vrd = root->calcNneigh(queryPoint, nNeigh);



}
