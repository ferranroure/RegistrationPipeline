#include "mykdtree.h"
#include "../../timer.h"

/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
myKdtree::myKdtree(){

    kdTree = NULL;
}

/* CONSTRUCTOR -----------------------------------------------------------
 *
 *      Given a Pipeline point cloud, ANN kdtree is created
 *
 *      @param P: is a vector of 3D point from our pipeline.
 */
myKdtree::myKdtree(vector<Point*> *P){

    ckdt = new converterKdtree();

    dataPts = ckdt->convertArray(P);

    kdTree = new ANNkd_tree(dataPts, (int)P->size(), DIMENSIONS);
}


/* DESTRUCTOR -----------------------------------------------------------
 *
 *      Using ANN deleting tools for memory cleaning.
 */
myKdtree::~myKdtree(){

    annDeallocPts(dataPts);
    delete kdTree;
    delete ckdt;
}

/* CALC N NEIGBOURS -----------------------------------------------------
 *
 *      Find nearest neigbour points of a given query point.
 *
 *      @param queryPoint: This method find the nearest neigbours of this point.
 *      @param nNeigh: Number of reported neighbours.
 *      @return a vector of NN points and its distances.
 */
vector<returnData> myKdtree::calcNneigh(Point *queryPoint, int nNeigh) {


    ANNpoint q = ckdt->convertPoint(queryPoint);

    if(kdTree->nPoints() > 0){

        nnIdx = new ANNidx[nNeigh];		        // allocate near neighbor indices
        dists = new ANNdist[nNeigh];			// allocate near neighbor dists

        kdTree->annkSearch(q, nNeigh, nnIdx, dists, ERROR);
    }
    else{

        cerr << "ERROR: The kdtree is empty!" << endl;
        exit(EXIT_FAILURE);
    }

    // returning container
    vector<returnData> ret;

    // Coping results.
    for (int i = 0; i < nNeigh; ++i) {

        returnData rd;
        rd.sqrDist = dists[i];
        rd.index = nnIdx[i];
        ret.push_back(rd);
    }

    // Free memory
    delete[] nnIdx;
    delete[] dists;
    annDeallocPt(q);

    return ret;
}

/* CALC ONE NN -----------------------------------------------
 *
 *      This method finds the nearest neigbor (one) of a given point. It
 *      call's annkSearch() method.
 *
 *      @param queryPoint: This method find the nearest neigbours of this point.
 *      @param errEps: Error distance for approximate searching.
 *      @return only the distance if it's less than dmax.
*/
returnData myKdtree::calcOneNN(Point *queryPoint, float errEps) {

    // distance
    double sqrDist = 0;

    ANNpoint q = ckdt->convertPoint(queryPoint);

    if(kdTree->nPoints() > 0){

        nnIdx = new ANNidx[1];						// allocate near neighbor indices
        dists = new ANNdist[1];						// allocate near neighbor dists

        kdTree->annkSearch(q, 1, nnIdx, dists, 0);
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
 *      This method finds the 2nd nearest neigbour (one) of a given point. It
 *      call's annkSearch() method.
 *      This method is used to find a nearest neigbor of a point excluding itself.
 *      This is because we need to find the real NN, used to calc MMD (Mean Minimum Distance)
 *      of a point cloud. The queryPoint, in this case, is from the same point cloud.
 *
 *      @param queryPoint: This method find the nearest neigbours of this point.
 *      @return the nearest neigbour point and its distance.
*/
returnData myKdtree::calcOwnNN(Point *queryPoint){

    // distance
    double sqrDist = 0;

    ANNpoint q = ckdt->convertPoint(queryPoint);

    if(kdTree->nPoints() > 0){

        nnIdx = new ANNidx[2];						// allocate near neighbor indices
        dists = new ANNdist[2];						// allocate near neighbor dists

        kdTree->annkSearch(q, 2, nnIdx, dists, 0);

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

void myKdtree::printStats() {

    ANNkdStats st;
    kdTree->getStats(st);

    cout << st.depth;
}
