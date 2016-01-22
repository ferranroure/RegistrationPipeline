#include "mykdtree.h"

/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
myKdtree::myKdtree(){

    kdTree = NULL;
}

myKdtree::myKdtree(vector<Point*> *P){

    int nnPts = P->size();

    dataPts = convertArrayForKdtree(P);         // Converting data from "points" to "dataPts"

    kdTree = new ANNkd_tree(dataPts, nnPts, DIMENSIONS);

//    kdTree = new ANNkd_tree(					// build search structure
//            dataPts,					// the data points
//            nPts,						// number of points
//            DIM);						// dimension of space

//    annDeallocPts(ddataPts);
}


/* DESTRUCTOR -----------------------------------------------------------
 *
 */
myKdtree::~myKdtree(){

//    cout << "destrueixo kdtree" << endl;
    annDeallocPts(dataPts);
    //annDeallocPts(queryPts);
    delete kdTree;
    annClose();
}


/* CONVERT ARRAY FOR KDTREE -----------------------------------------
 *
 *  This method is a bridge between our point storing format and
 *  ANNkdtree point storing format. Given a vector<Point>, it transforms
 *  the data to ANNpointArray with ANNpoints.
 */
ANNpointArray myKdtree::convertArrayForKdtree(vector<Point*> *P){

    ANNpointArray pa;
    pa = annAllocPts(P->size(), DIMENSIONS);
    for (int i = 0; i < P->size(); ++i) {

        pa[i][0] = P->at(i)->getX();
        pa[i][1] = P->at(i)->getY();
        pa[i][2] = P->at(i)->getZ();
    }

    return pa;
}


/* CONVERT POINT FOR KDTREE -----------------------------------------
 *
 *  This method tranform a given Point to an ANNpoint from ANNkdtree class
 */
ANNpoint myKdtree::convertPointForKdtree(Point *p)
{
    ANNpoint annP;
    annP = annAllocPt(3);
    annP[0] = p->getX();
    annP[1] = p->getY();
    annP[2] = p->getZ();

    return annP;
}


///* CREATE KDTREE ----------------------------------------------------
// *
// *  This method creates a ANNKdtree with a given vector<Point>. It call's
// *  ANNkd_tree() method.
// */
//void myKdtree::create(vector<Point*> *P){
//
//    nPts = P->size();
//
//    dataPts = convertArrayForKdtree(P);         // Converting data from "points" to "dataPts"
//
//    kdTree = new ANNkd_tree(					// build search structure
//                    dataPts,					// the data points
//                    nPts,						// number of points
//                    DIM);						// dimension of space
//
//    //annDeallocPts(ddataPts);
//
//}


vector<returnData> myKdtree::calcNneigh(Point *queryPoint, int nNeigh) {

    // distance
    double sqrDist = 0;

    ANNpoint q = convertPointForKdtree(queryPoint);

    if(kdTree->nPoints() > 0){

        nnIdx = new ANNidx[nNeigh];						// allocate near neighbor indices
        dists = new ANNdist[nNeigh];						// allocate near neighbor dists

        kdTree->annkSearch(q, nNeigh, nnIdx, dists, ERROR);
    }
    else{

        cerr << "ERROR: The kdtree is empty!" << endl;
        exit(EXIT_FAILURE);
    }

    vector<returnData> ret;

//    cout << "Query point id: " << queryPoint->getIndex() << endl;
    for (int i = 0; i < nNeigh; ++i) {

        returnData rd;
//        cout << nnIdx[i] << " " << dists[i] << endl;
        rd.sqrDist = dists[i];
        rd.index = nnIdx[i];
        ret.push_back(rd);
    }
//    cout << endl;

    delete[] nnIdx;
    delete[] dists;
    annDeallocPt(q);

    return ret;
}

/* CALC ONE NN -----------------------------------------------
 *
 *  This method finds the nearest neigbor (one) of a given point. It
 *  call's annkSearch() method.
 *  Returns only the distance if it's less than dmax.
*/
returnData myKdtree::calcOneNN(Point *queryPoint, float errEps) {

    // distance
    double sqrDist = 0;

    ANNpoint q = convertPointForKdtree(queryPoint);

    if(kdTree->nPoints() > 0){

        nnIdx = new ANNidx[1];						// allocate near neighbor indices
        dists = new ANNdist[1];						// allocate near neighbor dists

        kdTree->annkSearch(q, 1, nnIdx, dists, ERROR);

        sqrDist = dists[0];
    }
    else{

        cerr << "ERROR: The kdtree is empty!" << endl;
        exit(EXIT_FAILURE);
    }

    returnData rd;
    rd.sqrDist = sqrDist;
    rd.index = nnIdx[0];

    delete[] nnIdx;
    delete[] dists;
    annDeallocPt(q);

    return rd;
}


/*  CALC OWN NN -----------------------------------------------
 *
 *  This method finds the 2nd nearest neigbor (one) of a given point. It
 *  call's annkSearch() method.
 *  This method is used to find a nearest neigbor of a point excluding itself.
 *  This is because we need to find the real NN, used to calc MMD (Mean Minimum Distance)
 *  of a point cloud. The queryPoint, in this case, is from the same point cloud.
 *  Returns only the distance if it's less than dmax.
*/
returnData myKdtree::calcOwnNN(Point *queryPoint){

    // distance
    double sqrDist = 0;

    ANNpoint q = convertPointForKdtree(queryPoint);

    if(kdTree->nPoints() > 0){

        nnIdx = new ANNidx[2];						// allocate near neighbor indices
        dists = new ANNdist[2];						// allocate near neighbor dists

        kdTree->annkSearch(q, 2, nnIdx, dists, ERROR);

        sqrDist = dists[1];
    }
    else{

        cerr << "ERROR: The kdtree is empty!" << endl;
        exit(EXIT_FAILURE);
    }

    returnData rd;
    rd.sqrDist = sqrDist;
    rd.index = nnIdx[1];

    delete[] nnIdx;
    delete[] dists;
    annDeallocPt(q);

    return rd;
}


returnData myKdtree::findPair(Point *queryPoint, float dist) {

    // distance
    double sqrDist = 0;

    ANNpoint q = convertPointForKdtree(queryPoint);

    int cand = 0;

    if(kdTree->nPoints() > 0){

        cand = kdTree->annkFRSearch(q, dist*dist, 0, nnIdx, dists, ERROR);

        nnIdx = new ANNidx[cand];						// allocate near neighbor indices
        dists = new ANNdist[cand];						// allocate near neighbor dists

        kdTree->annkFRSearch(q, dist*dist, cand, nnIdx, dists, ERROR);

        sqrDist = dists[cand-1];
    }
    else{

        cerr << "ERROR: The kdtree is empty!" << endl;
        exit(EXIT_FAILURE);
    }

    returnData rd;
    rd.sqrDist = sqrDist;
    rd.index = nnIdx[cand-1];

    delete[] nnIdx;
    delete[] dists;
    annDeallocPt(q);

    return rd;
}
