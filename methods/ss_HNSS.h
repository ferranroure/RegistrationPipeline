//
// Created by yago on 16/06/21.
//

#ifndef PIPELINE_SS_HNSS_H
#define PIPELINE_SS_HNSS_H

#include "../ISearchingStrategy.h"
#include "../include/point.h"
#include "../include/Converters/converter4PCS.h"
//#include "../IDetection.h"
#include "det_HierarchicalNormalSpaceSampling.h"
//#include "data.h"

#include <vector>


class ss_HNSS : public ISearchingStrategy {
public:
    ss_HNSS();
    ss_HNSS(ISearchingStrategy *inner,det_HierarchicalNormalSpaceSampling *det);
    ~ss_HNSS();

    void setData(Data *d);
    double execute();

private:
    ISearchingStrategy *innerSearchStrategy; // When we get here the search strategy should already be
    det_HierarchicalNormalSpaceSampling *detAccess;
};


#endif //PIPELINE_SS_HNSS_H
