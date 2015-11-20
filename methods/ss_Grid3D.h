//
// Created by ferran on 29/07/15.
//

#ifndef SS_GRID3D_H
#define SS_GRID3D_H


#include "../ISearchingStrategy.h"

class ss_Grid3D : public ISearchingStrategy{

public:

    ss_Grid3D();
    ~ss_Grid3D();

    void setData(Data *d);
    void execute();
};


#endif //SS_GRID3D_H
