/*
 * Distance Connected Components detector
 *
 * This class is used to find connected components into the point cloud
 * but using distance values between points instead of edges.
 */

#ifndef DET_DISTCONNECTEDCOMPONENTS_H
#define DET_DISTCONNECTEDCOMPONENTS_H

#include "../IDetection.h"
#include "../include/DataStructures/kdtree/mykdtree.h"
#include <list>

#define NO_CAND -1
#define NO_NEIGH -2
#define GO_AHEAD -3

struct Component{

    vector<Point*> compPoints;
};

class det_DistConnectedComponents : public IDetection{

public:
    det_DistConnectedComponents();
    ~det_DistConnectedComponents();

    void setData(Data *d);
    void execute();

    vector<Component*> * findComponents(ElementSet *elem);
    int allVisited(const vector<bool> &cand);


};


#endif //DET_DISTCONNECTEDCOMPONENTS_H
