//
// Created by ferran on 17/06/15.
//

#include "ss_3PS.h"

ss_3PS::ss_3PS() {

    data = NULL;
}

ss_3PS::~ss_3PS() {

}

void ss_3PS::setData(Data *d) {

    data = d;
}

Base * ss_3PS::selectBase(ElementSet * es){

    float d = 0.0, dt;
    int n=es->nPoints();
    int i, at, bt, ct;
    float x;
    double qd = es->getDiagonal() * 0.2 * 2;
    Base *ret = NULL;

    // punt aleatori que serà fixat
    Point *a = es->getRandomPoint(data->params.useDetection);

    // Ho prova 1000 vegades.
    for (i=0;i<1000;i++)
    {
        // dos punts aleatoris dins el rang.
        Point *b = es->getRandomPoint(data->params.useDetection);
        Point *c = es->getRandomPoint(data->params.useDetection);
        // dos vectors entre els dos aleatoris i l'escollit abans.
        vector3D u(b->getX()-a->getX(), b->getY()-a->getY(), b->getZ()-a->getZ());
        vector3D w(c->getX()-a->getX(), c->getY()-a->getY(), c->getZ()-a->getZ());
        // La meitat de la distància del prod vectorial dels dos vectors.
        dt = 0.5 * (u.crossProduct(w)).modulus();

//        cout << "comprovem coses: " << endl;
//        cout << "dt: " << dt << "     d: " << d << endl;
//        cout << "qd: " << qd << endl;
//        cout << "|u|: " << u.modulus() << "   |w|: " << w.modulus() << endl << endl;


        // comprova que els punts estiguis prou wide. qd és un diametre tunejat que ve per paràmetre (depen de l'estimació de l'overlap crec).
        if (dt>d && u.modulus()<qd && w.modulus()<qd)
        {
            d=dt;
            if(ret!=NULL){
                delete ret;
            }
            ret = new Base(a, b, c);
        }
    }

    return ret;
}


Base *ss_3PS::selectOrthogonalBase(ElementSet *es, double thrs) {


    double min_dot = DBL_MAX;
    double maxArea = DBL_MIN;
    vector3D *abOK = NULL;
    vector3D *acOK = NULL;
    Point *a = NULL;
    Point *b = NULL;
    Point *c = NULL;
    bool found = false;
    int loops = 0;
    int max_loops = es->nPoints()*0.001;
    Base *res = NULL;

//    cout << max_loops << endl;
    while(loops < max_loops /*&& !found*/) {

        a = es->getRandomPoint(data->params.useDetection);
        b = es->findWidePoint(a, data->params.useDetection);


        // Computing the plane equation. (Ax + By + Cz = D)
        vector3D ab(b->getX() - a->getX(), b->getY() - a->getY(), b->getZ() - a->getZ());
        // searching D.
        float d = ab.getX() * a->getX() + ab.getY() * a->getY() + ab.getZ() * a->getZ();
        vector<Point *> cCand = es->findCoplanarPoints(ab, d, a, -1);

//        cout << cCand.size() << endl;
        for (int i = 0; i < cCand.size(); ++i) {

            Point *temp = cCand.at(i);

            vector3D ac(temp->getX() - a->getX(), temp->getY() - a->getY(), temp->getZ() - a->getZ());

            double dot = ab.dotProduct(ac);

//            double area = calcArea(a,b,temp);

            if (abs(dot) < thrs && a != temp && abs(dot) < min_dot) {
//                cout << "he trobat una base DOT: " << dot << endl;
                min_dot = abs(dot);

                if (res != NULL) { delete res; }
                res = new Base(a, b, temp);
                found = true;
            }
        }
        loops++;
    }


    if (found== false) cerr << "NO HE TROBAT EL C!! " << endl;

    return res;
}


vector<Base*> ss_3PS::findOrthogonalCandidates(Point *a, Point *b, ElementSet *es, float distAC){

    // Computing the plane equation. (Ax + By + Cz = D)
    vector3D norm(b->getX()-a->getX(), b->getY()-a->getY(), b->getZ()-a->getZ());

    // searching D.
    float d = norm.getX()*a->getX() + norm.getY()*a->getY() + norm.getZ()*a->getZ();

    vector<Point*> cCand = es->findCoplanarPoints(norm, d, a, distAC);

    vector<Base*> res;

    float thrs = 0.00001;

    int comptador =0;

    for (int i = 0; i < cCand.size(); ++i) {


        if(abs(a->dist(cCand.at(i)) - distAC) <= thrs) {

            comptador++;
            res.push_back(new Base(a, b, cCand.at(i)));
        }
    }

//    cout << cCand.size() << " " << comptador << endl;
    return res;
}


//vector<Base*> ss_3PS::findOrthogonalCandidates(Point *a, Point *b, ElementSet *es, float thrs, double area,
//                                               double beta, double gamma) {
//    Timer timer;
//    float acum = 0;
//
//    vector<Base*> BaseCand;
//
//    float thrs1 = 0.0001;
//    float thrs2 = 0.0001;
//
//    // vector ab, normal vector of the plane.
//    vector3D ab(b->getX()-a->getX(), b->getY()-a->getY(), b->getZ()-a->getZ());
//
//    for (int i = 0; i < es->nPoints(); ++i) {
//
//        Base *aux = NULL;
//
//        // find orthogonal point
////        Point *c = es->findWidePoint(b, data->params.useDetection);
//        Point *c = es->getPoint(i);
//
//        vector3D ac(c->getX() - a->getX(), c->getY() - a->getY(), c->getZ() - a->getZ());
//        vector3D bc(c->getX() - b->getX(), c->getY() - b->getY(), c->getZ() - b->getZ());
//
//        timer.reset();
//        if (abs(ab.dotProduct(ac)) < thrs1) {
//
//            aux = new Base(a,b,c);
//        }
//        else if (abs(ab.dotProduct(bc) < thrs1)) {
//
//            aux = new Base(b,a,c);
//        }
//        acum += timer.elapsed();
//
//        if(aux != NULL) {
//            double difArea = abs(aux->getArea() - area);
////            double difBeta = abs(aux->getBeta() - beta);
////            double difGamma = abs(aux->getGamma() - gamma);
//
////            cout << difArea << " " << difBeta << " " << difGamma << endl;
//
//            if (difArea < thrs2 /*&& difBeta < thrs2 && difGamma < thrs2*/) {
//
//                BaseCand.push_back(aux);
//            }
//            else{
//                delete aux;
//            }
//        }
//    }
//
////    cout << "he tardat. " << acum/es->nPoints() << " total: " << acum << endl;
//
//    return BaseCand;
//}



void ss_3PS::execute() {

    /*
     * ESQUEMA DE L'ALGORISME
     *
     * Buscar una Base de 2 punts wide
     * Trobar-ne les normals.
     * Buscar a Q la parella corresponent amb el mètode de sota.
     * Trobar moviment i comprovar-ho.
     *
     * IMPROTANT: Implementar el mètode per buscar els punts de Q que estan a una distància
     * d del punt p. Fer-ho amb ANN.
     */

    // DEFINE VARS
    int loops = 1;                  // # of tries with a base from A.
    int loopCount = 1;
    float THRS = 0.000001;//data->A->getMMD() * data->params.thrsFactor;           // Threshold to find bases.
    double minRes = DBL_MAX;                                                // Best residue of the movement.
    float percPairedPoints = 0;                                             // % of paired points between views.
    int nCand = 0;
    bool itsEnough = false;
    float pairedPointsTHRS = 0.25;



    // Normal computation. (Queda pendent no utilitzar pcl).
//    cout << "Computing normals" << endl;
//    data->A->calcNormals(data->A->getMMD() * data->params.radiusNormalFactor);
//    data->B->calcNormals(data->B->getMMD() * data->params.radiusNormalFactor);
//    cout << "Normals computed" << endl;


    while (!itsEnough && loopCount <= loops) {

        loopCount++;
        nCand = 0;

        cout << "NEW LOOP ------------------------------------------_" << endl;

        Timer timer;
        float acumTimeDist = 0;
        float acumTimeMot = 0;
        float acumTimeBest = 0;
        float acumTimeCand = 0;
        int counter = 0;
        int countDist = 0;
        int countMot = 0;

        // Select a base of 2 wide points.
        Base * A = selectOrthogonalBase(data->A, 0.000001);
        cout << "Base selection time: " << timer.elapsed() << endl; timer.reset();

        if (A==NULL) {
            cerr << "Fail selecting base" << endl;
            exit(EXIT_FAILURE);
        }

        Point *a = A->getPoint(0);
        Point *b = A->getPoint(1);
        Point *c = A->getPoint(2);

        data->crearteFileFromBase("/home/ferran/MEGA/PROJECTS/Pipeline/models/prova/0_base.ply", *a, *b, *c);
        

        vector<pair<Point*, Point*> > res = data->B->findPairsOctree(A->getDistAB(),0.0001);
        cout << "AB pairs size: " << res.size() << endl;
        cout << "Finding AB pairs: " << timer.elapsed() << endl; timer.reset();


//        vector<pair<Point*, Point*> > res2 = data->B->findPairs(A->getDistCA(), 0.0001);
//        cout << "AC pairs size: " << res2.size() << endl;
//        cout << "Finding AC pairs: " << timer.elapsed() << endl; timer.reset();


//        cout << "original points:" << endl;
//        A->print();
//        cout << "DistAB: " << distAB << " DistAC: " << distAC << " DistBC: " << distBC << endl;
//        cout << "------------------------" << endl;
    //        cout << "points found: " << endl;

        vector<Base*> BaseCand;



        for (int j = 0; j < res.size(); ++j) {


            Point *aa = res.at(j).first;
            Point *bb = res.at(j).second;
            Point *cc = NULL;

            // candidates to be point C.
            timer.reset();
            //vector<Base*> BaseCand = findOrthogonalCandidates(aa, bb, data->B, THRS, A->getArea(), A->getBeta(),A->getGamma());
            vector<Base*> BaseCand = findOrthogonalCandidates(aa, bb, data->B, A->getDistCA());
            acumTimeCand += timer.elapsed();



            if(!BaseCand.empty()){
//                cout << "# of candidates: " << BaseCand.size() << endl;
                nCand += BaseCand.size();


                for (int k = 0; k < BaseCand.size(); ++k) {

                    Base *B = BaseCand.at(k);

                    timer.reset();
                    bool okcheck = checkDistances(0.0001, A, B);
                    acumTimeDist += timer.elapsed();
                    timer.reset();
                    counter++;

                    if (counter < 4) {
                        stringstream sstm;
                        sstm << "models/prova/0_base" << counter << ".ply";
                        data->crearteFileFromBase(sstm.str(), *(B->getPoint(0)), *(B->getPoint(1)), *(B->getPoint(2)));
                    }

                    if(okcheck){
                        countDist++;
                        countMot++;
                        timer.reset();
                        data->cM = findBestMotion(0.000467944, A, B, minRes, percPairedPoints, data->cM, 0.001);
                        acumTimeBest += timer.elapsed();

                        if (percPairedPoints > pairedPointsTHRS) {itsEnough=true; break;}
                    }

                }
            }
            else{
//                cerr << "The thirth vector is NULL. Aborting..." << endl;
            }


//            if(a->getNormal() == ap->getNormal() || a->getNormal() == bp->getNormal()){
//                if(b->getNormal() == ap->getNormal() || b->getNormal() == bp->getNormal()){
//
//                    cout << "I found a pair" << endl;
//                    ap->print();
//                    cout << "normal: "; ap->getNormal()->write();
//                    bp->print();
//                    cout << "normal: "; bp->getNormal()->write();
//                }
//            }
        } //<-- AQUEST ES DEL FOOOOOR! :d

        cout << "# of orthogonal base candidates: " << nCand << endl;
        cout << "# of dist-ok base candidates: " << countDist << endl;
        cout << "Mean time to findCandidates: " << acumTimeCand/res.size() << " Total: " << acumTimeCand << endl;
        cout << "Mean time to checkDistance: " << acumTimeDist/counter << " Total: " << acumTimeDist <<  endl;
        cout << "Mean time to findBestMotion: " << acumTimeBest/counter  << " Total: " << acumTimeBest << endl;
        cout << "----------------------------------------------------------------------------------------------" << endl << endl;

//        for (int j = 0; j < res.size(); ++j) {
//
//            (res.at(j).first)->print();
//            cout << "normal: "; (res.at(j).first)->getNormal()->write();
//            (res.at(j).second)->print();
//            cout << "normal: "; (res.at(j).second)->getNormal()->write();
//            cout << endl << endl;
//        }


    }


}
