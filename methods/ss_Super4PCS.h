#ifndef PIPELINE_SS_SUPER4PCS_H
#define PIPELINE_SS_SUPER4PCS_H

#include "../ISearchingStrategy.h"
#include "../include/point.h"
#include "../include/Converters/converter4PCS.h"
#include <vector>
#include "../external/Super4PCS/super4pcs/shared4pcs.h"

//using namespace match_4pcs;
using namespace std;


class ss_Super4PCS : public ISearchingStrategy{

public:

    ss_Super4PCS();
    ~ss_Super4PCS();

    void setData(Data *d);
    void execute();
};


#endif //PIPELINE_SS_SUPER4PCS_H
