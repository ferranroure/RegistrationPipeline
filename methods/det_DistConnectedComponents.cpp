#include "det_DistConnectedComponents.h"


det_DistConnectedComponents::det_DistConnectedComponents() {

}

det_DistConnectedComponents::~det_DistConnectedComponents() {

}

void det_DistConnectedComponents::setData(Data *d) {

    data = d;
}

void det_DistConnectedComponents::execute() {

//    cout << "Detecting Connected Components" << endl;

    vector<Component*> *compA = findComponents(data->A);
    vector<Component*> *compB = findComponents(data->B);

    cout << "Components of A: " << compA->size() << endl;
    cout << "Components of B: " << compB->size() << endl;

    cout << "Points of A: " << data->A->nPoints() << endl;
    cout << "Points of B: " << data->B->nPoints() << endl;
//    exit(0);

    if(compA==NULL || compB==NULL) cerr << "DistConnectedComponents::No components found!" << endl;
}


vector<Component*> * det_DistConnectedComponents::findComponents(ElementSet *elem) {


    vector<Component*> *allCMP = new vector<Component*>();

    int status = NO_NEIGH;
    vector<bool> visited(elem->nPoints(), false);
    list<int> candidates;
    float dist = elem->getMMD()*20;
    Component *curr_comp = NULL;
    int nVisited = 0;
    int nNeigh = 50;



    while(status != NO_CAND) {

        if(nVisited >= elem->nPoints()){
            status = NO_CAND;
//            ostringstream num;   // stream used for the conversion
//            num << allCMP->size();
//            PlyIO plyio;
//            plyio.writeFile("../../models/bun"+num.str()+".ply", &(curr_comp->compPoints), false, true);
            break;
        }

        if (status == NO_NEIGH && candidates.empty()) {

//            if(curr_comp!=NULL){
//                ostringstream num;   // stream used for the conversion
//                num << allCMP->size();
//                PlyIO plyio;
//                plyio.writeFile("../../models/bun"+num.str()+".ply", &(curr_comp->compPoints), false, true);
//            }

            int id = allVisited(visited);
            candidates.push_back(id);
            visited.at(id) = true;
            ++nVisited;
            curr_comp = new Component();
            allCMP->push_back(curr_comp);
        }

        int p_id = candidates.front();
        candidates.pop_front();
        curr_comp->compPoints.push_back(elem->getPoint(p_id));



        // Create Kdtree and retrieve all neighbouring points at distance d.
        vector<returnData> vrd = elem->calcNneigh(elem->getPoint(p_id), nNeigh);

        vector<int> close_neigh;
        for (int i = 0; i < vrd.size(); ++i) {

            if(vrd.at(i).sqrDist <= dist*dist){

                close_neigh.push_back(vrd.at(i).index);
            }
        }
//        cout << "taken neigh: " << close_neigh.size() << endl;

        status = NO_NEIGH;

        for (int i = 1; i < close_neigh.size(); ++i) {

            int id_neigh = close_neigh.at(i);

            if (visited.at(id_neigh) == false) {

                candidates.push_back(id_neigh);
                visited.at(id_neigh) = true;
                ++nVisited;
//                cout << "\rVisited Points: " << nVisited << " # of candidates: " << candidates.size() << flush;
                status = GO_AHEAD;
            }

        }

    }

    int size = 0;
    int larger = -1;

    for (int j = 0; j < allCMP->size(); ++j) {

//        cout << allCMP->at(j)->compPoints.size() << endl;
        if(allCMP->at(j)->compPoints.size() > size) {

            larger = j;
            size = allCMP->at(j)->compPoints.size();
        }
    }

    elem->workpoints = &(allCMP->at(larger)->compPoints);

    return allCMP;
}

// Check if there are points to visit and return the first non-visit point. Otherwise return -1
int det_DistConnectedComponents::allVisited(const vector<bool> &cand){

    for (int i = 0; i < cand.size(); ++i) {

        if(cand.at(i) == false){
            return i;
        }
    }

    return NO_CAND;
}
