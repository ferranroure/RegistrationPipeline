//
// Created by ferran on 26/11/15.
//

#ifndef DET_KINECTSUPERSAMPLING_H
#define DET_KINECTSUPERSAMPLING_H

#include "../IDetection.h"
#include "../include/point.h"

class det_KinectSuperSampling : public IDetection {

public:
    det_KinectSuperSampling();
    ~det_KinectSuperSampling();

    void setData(Data *d);
    void execute();
    void superSampling(ElementSet *X);
};


#endif //DET_KINECTSUPERSAMPLING_H


