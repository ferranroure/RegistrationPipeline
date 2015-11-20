//
// Created by ferran on 26/06/15.
//

#include "det_RandomSampling.h"

det_RandomSampling::det_RandomSampling() {

    data = NULL;
}

det_RandomSampling::~det_RandomSampling() {

}

void det_RandomSampling::setData(Data *d) {

    data = d;
}

void det_RandomSampling::execute() {


    extractRandomSampling(data->A, data->params.nSamples);
    extractRandomSampling(data->B, data->params.nSamples);
}

void det_RandomSampling::extractRandomSampling(ElementSet *X, int sample) {

    srand(1962);
//    srand(time(NULL));
    X->workpoints = new vector<Point*>;

    int s1 = X->allpoints->size()/sample;

    for (int i=0;i<X->allpoints->size();i++)
    {
        if (rand()%s1==0)
        {
            X->workpoints->push_back(X->allpoints->at(i));
        }
    }
}
