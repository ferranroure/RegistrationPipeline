//
// Created by ferran on 18/01/16.
// THis class is both used for TriHas and CompressedOctree
//

#ifndef PIPELINE_ADAPTERDATASTRUCT_H
#define PIPELINE_ADAPTERDATASTRUCT_H

#include "../../../external/MoreStructures/AuxiliaryClasses/point3D.h"
#include "../../../external/MoreStructures/AuxiliaryClasses/Element.h"
#include "../../point.h"


class AdapterDataStruct {

public:

    AdapterDataStruct();
    ~AdapterDataStruct();

    vector<Element *> convertArray(vector<Point*> *P);


};


#endif //PIPELINE_ADAPTERDATASTRUCT_H
