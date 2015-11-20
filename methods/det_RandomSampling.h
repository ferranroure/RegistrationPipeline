//
// Created by ferran on 26/06/15.
//

#ifndef DET_RANDOMSAMPLING_H
#define DET_RANDOMSAMPLING_H

#include "../IDetection.h"


class det_RandomSampling : public IDetection  {

public:
    det_RandomSampling();
    ~det_RandomSampling();

    void setData(Data *d);
    void execute();

    void extractRandomSampling(ElementSet *X, int sample);
};


#endif //DET_RANDOMSAMPLING_H
