//
// Created by ferran on 18/01/16.
// THis class is both used for TriHash and CompressedOctree
//

#ifndef CONVERTERYAGO_H
#define CONVERTERYAGO_H

#include "../../external/MoreStructures/AuxiliaryClasses/point3D.h"
#include "../../external/MoreStructures/AuxiliaryClasses/Element.h"
#include "../point.h"


class converterYago {

public:

    converterYago();
    ~converterYago();

    vector<Element *> convertArray(vector<Point*> *P);
    point3D * convertPoint(Point *p);


};


#endif //CONVERTERYAGO_H
