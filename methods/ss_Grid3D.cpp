//
// Created by ferran on 29/07/15.
//

#include "ss_Grid3D.h"

ss_Grid3D::ss_Grid3D() {

    data = NULL;
}

ss_Grid3D::~ss_Grid3D() {

}

void ss_Grid3D::setData(Data *d) {

    data = d;
}

void ss_Grid3D::execute() {

    // initialize around the center of masses

    // Calc the diagonal and de bounding box of the object B.
    float digA = data->A->calcDiagonal();
    float xminA = data->A->getXmin();
    float xmaxA = data->A->getXmax();
    float yminA = data->A->getYmin();
    float ymaxA = data->A->getYmax();
    float zminA = data->A->getZmin();
    float zmaxA = data->A->getZmax();

    Point *cInit = new Point(xminA + (fabs(xmaxA - xminA)) / 2, yminA + (fabs(ymaxA - yminA)) / 2,
                         zminA + (fabs(zmaxA - zminA)) / 2);
    returnData rdInit = data->A->getDataStruct()->calcOneNN(cInit,data->A->getMMD() * data->params.nnErrorFactor);
    Point *pA;
    if (rdInit.index != -1) {
         pA = new Point(*(data->A->getPoint(rdInit.index)));
    }
    else
    {
        cout<<"FIL DE TUPLA no inicialitza"<<endl;
        exit(-1);
    }
    // data->A->getRandomPoint();                  // Use random point from A.


    cout<<"Point A initialised to "<<pA->getX()<<" "<<pA->getY()<<" "<<pA->getZ()<<" "<<endl;

//    Point *pA = data->A->getPoint(58);                    // Use specific point from A for the tests.


    int n = 1;
    int nCells = data->params.nCells;
    float cellsizeX = 0;
    float cellsizeY = 0;
    float cellsizeZ = 0;
    double minRes = DBL_MAX;
    motion3D *bestMotion = NULL;

    float percOfPoints = 0.01;
    float size = (float)data->A->getAllpoints()->size() * percOfPoints;

    // Calc the diagonal and de bounding box of the object B.
    float dig = data->B->calcDiagonal();
    float xmin = data->B->getXmin();
    float xmax = data->B->getXmax();
    float ymin = data->B->getYmin();
    float ymax = data->B->getYmax();
    float zmin = data->B->getZmin();
    float zmax = data->B->getZmax();

    int count = 0;
    int pairedPoints = 0;
    float percPairedPoints = 0;

    float factor = 1;
    float threshold = 0;
    float incFactor = 0;

    float maxThreshold = -1;
    float minThreshold = 0;

    // Setting different parameters for the execution.
    if(data->params.realData){

        threshold = 0.05;
        incFactor = 0.01;
        maxThreshold = 1.4;
        minThreshold = 0.005;
    }
    else if (data->params.useDescription){

        threshold = data->params.GTdescThrs;
        incFactor = ( (data->params.GTmaxDescDist-data->params.GTminDescDist)/50 );
        maxThreshold = data->params.GTmaxDescDist;
        minThreshold = data->params.GTdescThrs;
    }
/*
    cout << "Thrs: " << threshold << endl;
    cout << "incFactor: " << incFactor << endl;
    cout << "maxThr: " << maxThreshold << endl;
    cout << "minThr: " << minThreshold << endl;
*/
    // vector of cell centroids
    vector<Point> centroids;

int counter=0;

    while ( 1 ){

        int innerCounter=0;

        counter++;
        cout<<" looping "<<counter<<endl;
       int count2 = 0;

        bool validDesc = false;
        bool bestResidue = false;


        threshold = threshold * factor;

        cellsizeX = (xmax-xmin)/nCells;
        cellsizeY = (ymax-ymin)/nCells;
        cellsizeZ = (zmax-zmin)/nCells;

        float pxmin = xmin;
        float pymin = ymin;
        float pzmin = zmin;
        float pxmax, pymax, pzmax;
        float tymin = pymin;
        float tzmin = pzmin;

        // Exploring all the grid.
        for (int i=1; i<=nCells; i++){

            pxmax = pxmin + cellsizeX;
            pymin = tymin;

            for (int j=1; j<=nCells; j++){

                pymax = pymin + cellsizeY;
                pzmin = tzmin;

                for (int k=1; k<=nCells; k++) {

                    innerCounter++;
                 //   cout<<" inner loop "<<innerCounter<<endl;

                    pzmax = pzmin + cellsizeZ;

                    // getting center of cell.
                    Point *c = new Point(pxmin + (fabs(pxmax - pxmin)) / 2, pymin + (fabs(pymax - pymin)) / 2,
                                         pzmin + (fabs(pzmax - pzmin)) / 2);


                    // Saving centroids
                   // centroids.push_back(*c);

                    //getting the neighbour of centercell point.
                    returnData rd = data->B->getDataStruct()->calcOneNN(c,data->B->getMMD() * data->params.nnErrorFactor);
                    if (rd.index != -1)
                    {
                        Point pB(*(data->B->getPoint(rd.index)));

                     //   cout<<"Point B initialised to "<<pB.getX()<<" "<<pB.getY()<<" "<<pB.getZ()<<" "<<endl;


                        delete c;

                        if (pB.isInside(pxmin, pxmax, pymin, pymax, pzmin, pzmax)) {

                        count++;
                        count2++;
                        validDesc = true;

                        Point v = pA->distVector(pB);

                        // Copy of the moved element B to translate to A
                        //ElementSet copyOfB(*(data->B));
                        motion3D *motionAux = new motion3D(v.getX(), v.getY(), v.getZ());
                        //copyOfB.transform(motionAux);

                        int pairedPointsTemp = 0;

                       // double res = data->A->calcNN(copyOfB.getPoints(), percOfPoints, data->params.nnErrorFactor,pairedPointsTemp);
                       double res = data->A->calcNNMotion(data->B, percOfPoints, data->params.nnErrorFactor,pairedPointsTemp,motionAux);

                        float percPairedPointsTemp = (float) pairedPointsTemp / size;

                        if ((percPairedPointsTemp > percPairedPoints)) {

                            bestResidue = true;
                            minRes = res;
                            bestMotion = motionAux;
                            xmin = pxmin;
                            xmax = pxmax;
                            ymin = pymin;
                            ymax = pymax;
                            zmin = pzmin;
                            zmax = pzmax;
                            pairedPoints = pairedPointsTemp;
                            percPairedPoints = (float) pairedPoints / size;
                            cout << "Resiude: " << minRes << endl;
                            cout << "% of paired points: " << percPairedPoints * 100 << "%" << endl << endl;
                        }
                        else if ((percPairedPointsTemp <= percPairedPoints) &&
                                 (percPairedPointsTemp >= percPairedPoints * 0.98)) {

                            if (res < minRes) {
                                bestResidue = true;
                                minRes = res;
                                bestMotion = motionAux;
                                xmin = pxmin;
                                xmax = pxmax;
                                ymin = pymin;
                                ymax = pymax;
                                zmin = pzmin;
                                zmax = pzmax;
                                pairedPoints = pairedPointsTemp;
                                percPairedPoints = (float) pairedPoints / size;
                            }
                            else {
                                delete motionAux;
                            }
                        }
                        else {

                            delete motionAux;
                        }

                        //                        }

                        }
                    }
                    pzmin = pzmax;
                }
                pymin = pymax;
            }
            pxmin = pxmax;
        }

        // If none descriptor passes the tests, upgrade the threshold.
        if(!validDesc && threshold <= maxThreshold){

            factor += incFactor;
        }
        else{
            n++;
            threshold = minThreshold;

            if(!bestResidue){

                break;
            }

        }

    }

    // Printing centroids in a file.
  //  PlyIO plyio;
  //  plyio.writeFile("../models/testDesc/bust/centroids.ply", &centroids);

    data->cM = bestMotion;

//    // Printing the results of the execution. (TEST)
//    cout << data->params.descMethod << ";";
//    cout << data->params.infile << ";";
//    cout << data->A->points->size() << ";";
//    cout << pairedPoints << ";";
//    cout << (float)pairedPoints / (float)data->A->points->size() << ";";
//    cout << minRes << ";";
//    cout << nCells*nCells*nCells*n << ";";
//    cout << count << ";";

    // Printing the results of the execution.
    cout << "% of paired points: " << (float)pairedPoints / size * 100 << "%" << endl;
    cout << "Residue: " << minRes << endl;
    cout << "Num of total cells: " << nCells*nCells*nCells*n << endl; // REVISAR QUE AIXO ESTÀ MALAMENT!
    cout << "Num of explored cells: " << count << endl;
}

