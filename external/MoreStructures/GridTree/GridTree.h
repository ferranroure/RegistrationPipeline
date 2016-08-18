#ifndef PIPELINE_GRIDTREE_H
#define PIPELINE_GRIDTREE_H

#define TOLERANCE 0.0000000001

#include "myPoint.h"
#include "Cell.h"
#include <ANN/ANNperf.h>
#include <vector>


using namespace std;

class GridTree
{

    int slotsPerDimension; //number of equally spaced subdivisions in each dimension
    int thrsKdtree;             // Threshols of points for kdtree construction
    vector< vector<double> > limits; // three rows (x,y,z) and two columns(min,max), keeps the information on limits in each dimension

    vector<vector<vector<Cell *> > > grid;

    double tol; // tolerance to prevent numerical representation errors

    int nPoints;

    int meanPoints;                         // Mean of points of not empty cells.
    int notEmptyCells;

    double slotSizeX;
    double slotSizeY;
    double slotSizeZ;

    double sqSlotSizeX;
    double sqSlotSizeY;
    double sqSlotSizeZ;

public:

    GridTree(vector<myPoint *> &vec, int numC=-1, int _thrsKdtree=100);
    ~GridTree();

    void kdtreezation();                        // creartes kdtree in each cell with nPoints > certain number.
    int getNumElems();
    int getSlotsPerDimension();
    float getMeanHeight();
    void calcMeanPoints();                      // Compute mean of points of not empty cells.

    int findSlot(double val, char type, bool checkOutOfBounds=false, bool squared=false); // type=x,y,z returns the slot (for the givenn) where value val falls into. "checkOutOfBounds" indictes if we get out of bonds querys back IN bounds or if we throw an exception.

    vector<int> slotsTouched(double min, double max, char type, bool squared=false); // returns minimum and maximum slots touched by an interval in a dimension x,y o z (indicated by type)
    int numberSlotsTouchedSquared(double sqEps,  char type); // returns the number of slots touched up or down (receives the part outside of the current cell), then need to account for minimum and maximum slots


    vector<myPoint *> neighbors(myPoint *p, double eps); // returns all neigbors at distance at most eps from p, if it finds p it does not return it
    myPoint * oneNeighbor(myPoint *p, double eps); // returns all neigbors at distance at most eps from p, if it finds p it does not return it

};

#endif
