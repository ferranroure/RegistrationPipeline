//
// Created by yago on 16/09/08.
//

#include "ss_ImprovedGrid.h"

float boxMatching(float xMinA, float xMaxA,float yMinA, float yMaxA,float zMinA, float zMaxA,float xMinB, float xMaxB,float yMinB, float yMaxB,float zMinB, float zMaxB)
{

    float xMinINT=(xMinA>xMinB)?xMinA:xMinB;
    float xMaxINT=(xMaxA<xMaxB)?xMaxA:xMaxB;
    float yMinINT=(yMinA>yMinB)?yMinA:yMinB;
    float yMaxINT=(yMaxA<yMaxB)?yMaxA:yMaxB;
    float zMinINT=(zMinA>zMinB)?zMinA:zMinB;
    float zMaxINT=(zMaxA<zMaxB)?zMaxA:zMaxB;

    //cout<<"FIRST BOX X: "<<xMinA<<","<<xMaxA<<" y: "<<yMinA<<","<<yMaxA<<" z: "<<zMinA<<","<<zMaxA<<endl;
  //  cout<<"SECOND BOX X: "<<xMinB<<","<<xMaxB<<" y: "<<yMinB<<","<<yMaxB<<" z: "<<zMinB<<","<<zMaxB<<endl;
   // cout<<"INTERSECTION BOX X: "<<xMinINT<<","<<xMaxINT<<" y: "<<yMinINT<<","<<yMaxINT<<" z: "<<zMinINT<<","<<zMaxINT<<endl;

    if(xMinINT>xMaxINT||yMinINT>yMaxINT||zMinINT>zMaxINT)return 0;
    else return ( (xMaxINT-xMinINT)*(yMaxINT-yMinINT)*(zMaxINT-zMinINT) );


}


ss_ImprovedGrid::ss_ImprovedGrid() {

    data = NULL;

}

ss_ImprovedGrid::~ss_ImprovedGrid() {

}

void ss_ImprovedGrid::setData(Data *d) {

    data = d;

}

void ss_ImprovedGrid::execute() {

    cout<<"IMP=ROVED GRID!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;

    // initialize around the center of masses

    // Calc the diagonal and de bounding box of the object B.
    float digA = data->A->calcDiagonal();
    float xminA = data->A->getXmin();
    float xmaxA = data->A->getXmax();
    float yminA = data->A->getYmin();
    float ymaxA = data->A->getYmax();
    float zminA = data->A->getZmin();
    float zmaxA = data->A->getZmax();

    // also compute the goal volume
    double goalVolume=(xmaxA-xminA)*(ymaxA-yminA)*(zmaxA-zminA);

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

    // now for set B

    // Calc the diagonal and de bounding box of the object B.
    float dig = data->B->calcDiagonal();
    float xmin = data->B->getXmin();
    float xmax = data->B->getXmax();
    float ymin = data->B->getYmin();
    float ymax = data->B->getYmax();
    float zmin = data->B->getZmin();
    float zmax = data->B->getZmax();

    delete cInit;
    cInit = new Point(xminA + (fabs(xmax - xmin)) / 2, ymin + (fabs(ymax - ymin)) / 2,
                      zmin + (fabs(zmax - zmin)) / 2);
    returnData rdInit2 = data->B->getDataStruct()->calcOneNN(cInit,data->B->getMMD() * data->params.nnErrorFactor);
    Point *pB;
    if (rdInit2.index != -1) {
        pB = new Point(*(data->B->getPoint(rdInit2.index)));
    }
    else
    {
        cout<<"B FIL DE TUPLA no inicialitza"<<endl;
        exit(-1);
    }

    cout<<"Point B center of masses is at  "<<pB->getX()<<" "<<pB->getY()<<" "<<pB->getZ()<<" "<<endl<<endl<<endl<<endl<<endl;

    double bestBoxOverlap=100*boxMatching(xminA, xmaxA, yminA, ymaxA,zminA, zmaxA, xmin, xmax, ymin, ymax,zmin, zmax)/goalVolume;
    cout<<" FIRST BOX overlap area % "<<bestBoxOverlap<<endl;

    double bestGridMatchedPercentage =0;

    //parameters
    int cellSizeFactor=1;
    int repeats=4;

//    int count = 0;
    int pairedPoints = 0;
    float percPairedPoints = 0;
    float matchedThreshold = 0.1;

    // Build grids for the two sets
   // Trihash *tA = new TriHash();
    myTriHash *tA= new myTriHash(data->A->getWorkpoints(), digA);
    myTriHash *tB= new myTriHash(data->B->getWorkpoints(), dig);

    int n = 1;
    int nCells = data->params.nCells; // should de divisible by 2
    float cellSizeX = (xmax-xmin)/(cellSizeFactor*nCells);
    float cellSizeY = (ymax-ymin)/(cellSizeFactor*nCells);
    float cellSizeZ = (zmax-zmin)/(cellSizeFactor*nCells);

    double minRes = DBL_MAX;
    motion3D *bestMotion = NULL;

    float percOfPoints = 0.01;
    float size = (float)data->A->getAllpoints()->size() * percOfPoints;
    bool improved;

    cout<<"GOING TO START LOOP WITH NCELLS "<<nCells<<endl;
    int count=0;

    double currentX,currentY,currentZ;
    while ( percPairedPoints < matchedThreshold && repeats >0 ) {

        cout<<percPairedPoints<<" paired in repeat "<<repeats<<" CELL sizes X "<<cellSizeX<<" y "<<cellSizeY<<" Z  "<<cellSizeZ<<" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;

        Point *placeholder = nullptr;
        improved = false;

        for (int i = 0; i < nCells; i++) {

            currentX = pB->getX() + cellSizeX / 2 - ((cellSizeX * nCells / 2)) + i * cellSizeX;
            for (int j = 0; j < nCells; j++) {
                currentY = pB->getY() + cellSizeY / 2 - ((cellSizeY * nCells / 2)) + j * cellSizeY;

                for (int k = 0; k < nCells; k++) {
                    currentZ = pB->getZ() + cellSizeZ / 2 - ((cellSizeZ * nCells / 2)) + k * cellSizeZ;

                    count++;
                    // get current point and translation vector
                    Point c = Point(currentX, currentY, currentZ);
                    //cout << "Point in B set to " << c.getX() << " " << c.getY() << " " << c.getZ() << " " << endl;
                    Point v = pA->distVector(c);

                    double currentBoxOverlap=100*boxMatching(xminA, xmaxA, yminA, ymaxA,zminA, zmaxA, xmin+v.getX(), xmax+v.getX(), ymin+v.getY(), ymax+v.getY(),zmin+v.getZ(), zmax+v.getZ())/goalVolume;
                    //cout<<" CURRENT BOX overlap area % "<<currentBoxOverlap<<endl<<endl;

                    motion3D *motionAux= nullptr;
                    motionAux = new motion3D(v.getX(), v.getY(), v.getZ());

                    if(currentBoxOverlap>bestBoxOverlap||(repeats==1)) {
                       // cout<<" entering overlap area % "<<currentBoxOverlap<<" so far did loops: "<<count<<" "<<i<<" "<<j<<" "<<k<<endl<<endl;

                        if((repeats==1)&&(currentBoxOverlap>(bestBoxOverlap*0.75))) {

                            int pairedPointsTemp = 0;
                            double res = data->A->calcNNMotion(data->B, percOfPoints, data->params.nnErrorFactor,pairedPointsTemp,motionAux);
                            float percPairedPointsTemp = (float) pairedPointsTemp / size;

                            if ((percPairedPointsTemp > percPairedPoints)) {

                           //     cout << " IMPROVING CURRENT BOX overlap area % " << currentBoxOverlap << endl << endl;
                                minRes = res;
                                bestMotion = motionAux;
                                pairedPoints = pairedPointsTemp;
                                percPairedPoints = (float) pairedPoints / size;
                             //   cout << "Resiude: " << minRes << endl;
                              //  cout << "% of paired points: " << percPairedPoints * 100 << "%" << endl << endl;
                                if(currentBoxOverlap>bestBoxOverlap) bestBoxOverlap = currentBoxOverlap;
                              //  cout << "bestBoxOverlap " << bestBoxOverlap << endl;

                                improved = true;
                                if (placeholder != nullptr)delete placeholder;
                                placeholder = new Point(c.getX(), c.getY(), c.getZ());
                            }
                        }
                        else if ((repeats==2)&&(currentBoxOverlap>(bestBoxOverlap*0.75)))
                        {
                            double gridMatchedPercentage=tA->gridMatchPercentage(motionAux,tB);
                            if(gridMatchedPercentage>bestGridMatchedPercentage){
                                bestMotion = motionAux;
                                bestGridMatchedPercentage = gridMatchedPercentage;
                            }

                        }
                        else if (currentBoxOverlap>bestBoxOverlap)
                        {
                          //  cout << "ELSE IMPROVING CURRENT BOX overlap area % " << currentBoxOverlap << endl << endl;

                            bestMotion = motionAux;
                            bestBoxOverlap = currentBoxOverlap;
                          //  cout << "bestBoxOverlap " << bestBoxOverlap << endl;
                        }
                    }
                    else {

                        if(motionAux!= nullptr)delete motionAux;
                    }
                }
            }
        }


        if (improved) {
            delete pB;
            pB = new Point(placeholder->getX(), placeholder->getY(), placeholder->getZ());
            delete placeholder;
        }
        cellSizeX = cellSizeX / 2;
        cellSizeY = cellSizeY / 2;
        cellSizeZ = cellSizeZ / 2;
        repeats--;
    }

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
//    cout << "Num of explored cells: " << count << endl;


}
