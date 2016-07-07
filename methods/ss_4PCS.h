#ifndef SS_4PCS_H
#define SS_4PCS_H


#include "../ISearchingStrategy.h"
#include "../include/point.h"
#include "../include/Converters/converter4PCS.h"

#include <vector>

class ss_4PCS : public ISearchingStrategy {

private:
    float thr; // THRESHOLD FOR THE bestf
    int n_points; // Número de punts que consideren. Més o menys, pq en fan un tractament random raro.
    float norm_diff; //30     // Diferències entre les normals. No sé perquè ho fan servir.
    // This parameter is the multiplication factor applied to the threshold for corresponding searching.
    // p = q  if( dist(p,q) < (meanDistOfAllPoints * 2 ) * delta = eps; also called eps.
    float delta;
    // This parameter is used to select the 4th point in de base from A. Is provided in order to not select a point
    // which its correspondence in Q falls in a non-overlapping area.
    // qd = diam*overlap*2.0; -> this qd is used to find a wide point. length(u)< qd.
    float overlap;

public:

    ss_4PCS();
    ~ss_4PCS();

    void setData(Data *d);
    double execute();
};


#endif //SS_4PCS_H
