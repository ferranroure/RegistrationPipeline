#include "ss_SmartForce.h"

ss_SmartForce::ss_SmartForce(){

    data = NULL;
}

ss_SmartForce::~ss_SmartForce() {

}

void ss_SmartForce::setData(Data *d) {

    data = d;
}

void ss_SmartForce::execute() {


//    cout << "SSSSSSSSSSSSSSSSS" << endl;
//    cout << "SS::desc av. " <<  data->A->getWorkpoints()->at(0)->availableDescriptor() << endl;
//    data->A->getWorkpoints()->at(0)->descriptor->print();

    // Setting parameters
    float distThrs = data->A->getMMD() * data->params.thrsFactor;           // Threshold to find bases.
    int failCounter = 0;                                                    // Counter for failing tests
    int nloops = 10;                                                        // Number of tries
    int nNeigh = data->params.nNeighbours;                                  // # of candidates extracted from B.
    double minRes = DBL_MAX;                                                // Best residue of the movement.
    float percPairedPoints = 0;                                             // % of paired points between views.

    Base *BB = new Base();

    for (int ii=1; ii<=nloops; ii++){


        vector<Point*> *vbi, *vbj, *vbk;

        Base *BA = data->A->getRandomBase(data->params.useDetection);

        // If description is used, we search candidates according descriptor values.
        if(data->params.useDescription) {

            vbi = data->B->findCorrespondences(BA->getPoint(0), nNeigh, false, data->params.descMethod);
            vbj = data->B->findCorrespondences(BA->getPoint(1), nNeigh, false, data->params.descMethod);
            vbk = data->B->findCorrespondences(BA->getPoint(2), nNeigh, false, data->params.descMethod);
        }
        // If not, we use 500 ramdom points as candidates.
        else{
            nNeigh = 100;
            vbi = new vector<Point*>();
            vbj = new vector<Point*>();
            vbk = new vector<Point*>();
            for (int i = 0; i < nNeigh; ++i) {

                vbi->push_back(data->B->getRandomPoint(data->params.useDetection));
                vbj->push_back(data->B->getRandomPoint(data->params.useDetection));
                vbk->push_back(data->B->getRandomPoint(data->params.useDetection));
            }
        }

        // Find correspondence group of points for each query point.
        for(int i=0; i<nNeigh; i++){

            BB->addPoint(vbi->at(i));
            for(int j=0; j<nNeigh; j++){

                BB->addPoint(vbj->at(j));
                for(int k=0; k<nNeigh; k++){

                    BB->addPoint(vbk->at(k));

                    if (! BB->existRepeatedPoints() && checkDistances(distThrs, BA, BB)){

                        data->cM = findBestMotion(distThrs, BA, BB, minRes, percPairedPoints, data->cM, 1);

                        if (percPairedPoints > 0.95) break;
                    }
                    else{
                        failCounter++;
                    }

                    BB->removeLastPoint();
                }
                BB->removeLastPoint();
                if (percPairedPoints > 0.95) break;
            }
            BB->removeLastPoint();
            if (percPairedPoints > 0.95) break;
        }


        BA->clear();
        if (percPairedPoints > 0.95) break;

    }

    cout << "Resiude: " << minRes << endl;
    cout << "% of paired points: " << percPairedPoints * 100 << "%" << endl;

    //cout << percPairedPoints * 100 << "%;" << minRes << ";";

    //cout << "N: " << nNeigh*nNeigh*nNeigh << endl;
    //cout << "nfails by checkDist: " << failCounter << endl;

    //exit(0);

}

