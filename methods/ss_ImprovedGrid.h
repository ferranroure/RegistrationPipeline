//
// Created by yago on 16/09/08.
//

#ifndef PIPELINE_SS_IMPROVEDGRID_H
#define PIPELINE_SS_IMPROVEDGRID_H

#include "../ISearchingStrategy.h"
//#include "../external/MoreStructures/TriHash/TriHash.h"


class ss_ImprovedGrid : public ISearchingStrategy {

public:
    ss_ImprovedGrid();
    ~ss_ImprovedGrid();

    void setData(Data *d);
    void execute();
};


#endif //PIPELINE_SS_IMPROVEDGRID_H
