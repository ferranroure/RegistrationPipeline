#include "myPoint.h"
#include "Cell.h"
#include <ANN/ANNperf.h>
#include <vector>

#ifndef _GRIDTREE_
#define _GRIDTREE_

using namespace std;

class GridTree
{

    int slotsPerDimension; //number of equally spaced subdivisions in each dimension
    vector< vector<double> > limits; // three rows (x,y,z) and two columns(min,max), keeps the information on limits in each dimension

    vector<vector<vector<Cell *> > > grid;

    double tol; // tolerance to prevent numerical representation errors

    int nPoints;

public:

    GridTree(vector<myPoint *> &vec, int numC=-1, double iTol=0.0000000001);
    ~GridTree();

    void kdtreezation();                        // creartes kdtree in each cell with nPoints > certain number.
    int getNumElems();
    int getSlotsPerDimension();
    float getMeanHeight();

    int findSlot(double val, char type, bool checkOutOfBounds=false); // type=x,y,z returns the slot (for the givenn) where value val falls into. "checkOutOfBounds" indictes if we get out of bonds querys back IN bounds or if we throw an exception.
    int sqrFindSlot(double val, char type, bool checkOutOfBounds=false); // type=x,y,z returns the slot (for the givenn) where value val falls into. "checkOutOfBounds" indictes if we get out of bonds querys back IN bounds or if we throw an exception.

    vector<int> slotsTouched(double min, double max, char type); // returns minimum and maximum slots touched by an interval in a dimension x,y o z (indicated by type)
    vector<int> sqrSlotsTouched(double min, double max, char type); // returns minimum and maximum slots touched by an interval in a dimension x,y o z (indicated by type)

    vector<myPoint *> neighbors(myPoint *p, double eps); // returns all neigbors at distance at most eps from p, if it finds p it does not return it
    vector<myPoint *> oneNeighbor(myPoint *p, double eps); // returns all neigbors at distance at most eps from p, if it finds p it does not return it

};

#endif
