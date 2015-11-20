#include "ISearchingStrategy.h"



ISearchingStrategy::ISearchingStrategy() {

}

ISearchingStrategy::~ISearchingStrategy() {

}

motion3D * ISearchingStrategy::findBestMotion(float thrs, Base *BA, Base *BB, double &minRes, float &percPairedPoints, motion3D *bestMotion, float percOfPoints){

    int pairedPoints = 0;
    //float percPairedPoints = 0;
    int pairedPointsTemp = 0;
    //motion3D *bestMotion = NULL;
    //double minRes = DBL_MAX;

    Point *ai = BA->getPoint(0);
    Point *aj = BA->getPoint(1);
    Point *ak = BA->getPoint(2);
    Point *bi = BB->getPoint(0);
    Point *bj = BB->getPoint(1);
    Point *bk = BB->getPoint(2);

    // Size to divide that depends on the percOfPoints param.
    float size = (float)data->A->getAllpoints()->size() * percOfPoints;

    // Copy of the moved element B to translate to A
    ElementSet aux(*(data->B));


    try{
        motion3D *motionAux = new motion3D(*bi, *bj, *bk, *ai, *aj, *ak, thrs);

        aux.transform(motionAux);

        double res = data->A->calcNN(aux.getPoints(),percOfPoints, data->params.nnErrorFactor, pairedPointsTemp);

        float percPairedPointsTemp = (float)pairedPointsTemp / size;

        if( (percPairedPointsTemp > percPairedPoints)){

            minRes = res;
            bestMotion = motionAux;
            pairedPoints = pairedPointsTemp;
            percPairedPoints = (float)pairedPoints / size;

            cout << "Resiude: " << minRes << endl;
            cout << "% of paired points: " << percPairedPoints * 100 << "%" << endl << endl;

        }
        else if( (percPairedPointsTemp <= percPairedPoints) && (percPairedPointsTemp >= percPairedPoints*0.98) ){

            if(res < minRes){

                minRes = res;
                bestMotion = motionAux;
                pairedPoints = pairedPointsTemp;
                percPairedPoints = (float)pairedPoints / size;

                cout << "* Resiude: " << minRes << endl;
                cout << "* % of paired points: " << percPairedPoints * 100 << "%" << endl << endl;

            }
            else{
                delete motionAux;
            }
        }
        else delete motionAux;

    }
    catch (const char* msg) {
        cerr << msg << endl;
    }


    return bestMotion;
}

/* FIND CORRESPONDENCE --------------------------------------------------------------
 *
 *  This method finds a correspondence point of a given point, in a given elementSet.
 */
Point * ISearchingStrategy::findCorrespondence(Point *p, ElementSet *es){

    Point *q = NULL;

    if(p->getDescriptor()->isAvailable()){

        float minDist = FLT_MAX;
        int i = 0;
        int c = 0;
        for(vector<Point>::iterator it = es->getPoints()->begin(); it!=es->getPoints()->end(); ++it){
            i++;
            float dist = p->getDescriptor()->compare(it->getDescriptor());

            if(dist < minDist){
                c = i;
                q = &(*it);
                minDist = dist;
                //cout << "MINDIST: " << minDist << endl;
            }
        }
        //cout << "c: " << c <<  " i: " << i << ", es.size: " << es->points->size() << endl;
    }
    else{

        cerr << "SS: No descriptor availabe for search for correspondences." << endl;
    }

    return q;
}


/* FIND WIDE POINT --------------------------------------------------------------
 *
 *  This method finds a partner point from an ElementSet which is located far
 *  (more than diagonal/2) from a certain point.
 */
//Point * ISearchingStrategy::findWidePoint(Point *p, ElementSet *es){
//
//    double minDist = (es->calcDiagonal()) / 4;
//    double dist = 0;
//    Point *q = NULL;
//
//    do{
//        q = es->getRandomPoint();
//        dist = p->dist(q);
//    }while(dist < minDist);
//
//    return q;
//}

bool ISearchingStrategy::checkNormals(Base *A, Base *B) {




}

bool ISearchingStrategy::checkMotion(Base *A, Base *B) {

    int nNeigh = 5 ;
    bool ok = true;
    double thrs = 0.001; //data->params.MMD * 0.5;

    vector<returnData> a1Neigh = data->A->calcNneigh(A->getPoint(0), nNeigh);
    vector<returnData> a2Neigh = data->A->calcNneigh(A->getPoint(1), nNeigh);
    vector<returnData> a3Neigh = data->A->calcNneigh(A->getPoint(2), nNeigh);

    vector<returnData> b1Neigh = data->B->calcNneigh(B->getPoint(0), nNeigh);
    vector<returnData> b2Neigh = data->B->calcNneigh(B->getPoint(1), nNeigh);
    vector<returnData> b3Neigh = data->B->calcNneigh(B->getPoint(2), nNeigh);

    Point *a1 = A->getPoint(0);
    Point *a2 = A->getPoint(1);
    Point *a3 = A->getPoint(2);
    Point *b1 = B->getPoint(0);
    Point *b2 = B->getPoint(1);
    Point *b3 = B->getPoint(2);

    for (int i = 1; i < nNeigh; ++i) {

//        cout << abs(a1Neigh.at(i).sqrDist - b1Neigh.at(i).sqrDist) << endl;
//        cout << a1Neigh.at(i).sqrDist << " " << b1Neigh.at(i).sqrDist << endl;
//        cout << a1Neigh.at(i).index << " " << b1Neigh.at(i).index << endl << endl;


        Point *a1n = data->A->allpoints->at(a1Neigh.at(i).index);
        Point *a2n = data->A->allpoints->at(a2Neigh.at(i).index);
        Point *a3n = data->A->allpoints->at(a3Neigh.at(i).index);

        Point *b1n = data->B->allpoints->at(b1Neigh.at(i).index);
        Point *b2n = data->B->allpoints->at(b2Neigh.at(i).index);
        Point *b3n = data->B->allpoints->at(b3Neigh.at(i).index);

//        cout << abs(a1->dist(a2n) - b1->dist(b2n)) << endl;
//        cout << abs(a1->dist(a3n) - b1->dist(b3n)) << endl;
//        cout << abs(a2->dist(a3n) - b2->dist(b3n)) << endl << endl;

        if(abs(a1->dist(a2n) - b1->dist(b2n)) > thrs) {ok = false; break;}
        if(abs(a1->dist(a3n) - b1->dist(b3n)) > thrs) {ok = false; break;}
        if(abs(a2->dist(a3n) - b2->dist(b3n)) > thrs) {ok = false; break;}

//        if(abs(a1Neigh.at(i).sqrDist - b1Neigh.at(i).sqrDist) > thrs) {ok = false; break;}
//        if(abs(a2Neigh.at(i).sqrDist - b2Neigh.at(i).sqrDist) > thrs) {ok = false; break;}
//        if(abs(a3Neigh.at(i).sqrDist - b3Neigh.at(i).sqrDist) > thrs) {ok = false; break;}
//
//        if(abs(a1Neigh.at(i).sqrDist - b1Neigh.at(i).sqrDist) > thrs) {ok = false; break;}
//        if(abs(a1Neigh.at(i).sqrDist - b1Neigh.at(i).sqrDist) > thrs) {ok = false; break;}
//        if(abs(a1Neigh.at(i).sqrDist - b1Neigh.at(i).sqrDist) > thrs) {ok = false; break;}
    }

//    if(!ok) cout << "cabroooo " << endl;
    return ok;
}

/* CHECK DISTANCES --------------------------------------------------------------
 *
 *  This method checks if the distances between points at base A are the same at base B.
 */
bool ISearchingStrategy::checkDistances(float thrs, Base *A, Base *B){

    int p = 0;
    int q = 1;
    bool stop = false;

    Point *ap = NULL;
    Point *aq = NULL;
    Point *bp = NULL;
    Point *bq = NULL;

//    A->print();
//    B->print();


    while(!stop && q<B->size()){

        ap = A->base.at(p);
        aq = A->base.at(q);
        bp = B->base.at(p);
        bq = B->base.at(q);

        /* ap->printWithDesc();
         bp->printWithDesc();
         aq->printWithDesc();
         bq->printWithDesc();
 */


//        cout << "ap-aq: " << ap->dist(*aq) << " bp-bq: " << bp->dist(*bq) << endl;
//        cout << "dists: " << abs( ap->dist(aq) - bp->dist(bq) ) << endl << endl;

        if(abs( ap->dist(aq) - bp->dist(bq) ) > thrs){

            stop = true;
        }

        p++;
        q++;

        if (q==B->size() && B->size()>2 && stop==false){

            // Check the last with the first
            ap = A->base.at(A->size()-1);
            aq = A->base.at(0);
            bp = B->base.at(B->size()-1);
            bq = B->base.at(0);

//            cout << "*dists: " << abs( ap->dist(aq) - bp->dist(bq) ) << endl << endl;

            if(abs( ap->dist(aq) - bp->dist(bq) ) > thrs ){

                stop = true;
            }
        }
    }

//    if(!stop) cout << "he passat!" << endl;

    return !stop;
}

bool ISearchingStrategy::checkDescriptors(float thrs, Point *ai, Point *aj, Point *ak, Point *bi, Point *bj, Point *bk){

    if (ai->getDescriptor()->compare(bi->getDescriptor()) < thrs &&
            aj->getDescriptor()->compare(bj->getDescriptor()) < thrs &&
            ak->getDescriptor()->compare(bk->getDescriptor()) < thrs){

        return true;
    }
    else{
        return false;
    }
}


double ISearchingStrategy::calcArea(Point *a, Point *b, Point *c) {

    double ab = a->dist(b);
    double bc = b->dist(c);
    double ca = c->dist(a);

    double s = (ab+bc+ca)/2;

    return sqrt(s*(s-ab)*(s-bc)*(s-ca));
}
