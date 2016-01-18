//
// Created by ferran on 17/06/15.
//

#ifndef SS_3PS_H
#define SS_3PS_H


#include "../ISearchingStrategy.h"
#include "../include/point.h"
#include "../include/timer.h"
#include "../include/plyio.h"
#include <string>
#include "../include/DataStructures/octree/Octree.h"


class ss_3PS : public ISearchingStrategy{

public:

    ss_3PS();
    ~ss_3PS();

    void setData(Data *d);
    void execute();

    Base * selectBase(ElementSet * es);
    Base *selectOrthogonalBase(ElementSet *es, double thrs);
    vector<Base*> findOrthogonalCandidates(Point *a, Point *b, ElementSet *es, float thrs, double area,
                                           double beta, double gamma);

    vector<Base*> findOrthogonalCandidates(Point *a, Point *b, ElementSet *es, float distAC);

};


#endif //SS_3PS_H
