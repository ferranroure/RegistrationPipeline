#ifndef SS_4PCS_H
#define SS_4PCS_H


#include "../ISearchingStrategy.h"
#include "../include/point.h"
#include "../include/Converters/converter4PCS.h"
#include "../include/timer.h"

#include <vector>

class ss_4PCS : public ISearchingStrategy {

public:

    ss_4PCS();
    ~ss_4PCS();

    void setData(Data *d);
    void execute();
};


#endif //SS_4PCS_H
