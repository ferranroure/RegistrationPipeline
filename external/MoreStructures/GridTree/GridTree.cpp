#include "GridTree.h"

GridTree::GridTree(vector<myPoint*> &vec, int numC, int _thsPoints, double iTol)
{
    slotsPerDimension = numC;
    tol=iTol;
    thsPoints = _thsPoints;

    nPoints = vec.size();

    if(slotsPerDimension==-1) // in the case where the number of cells was not specified, make charge factor to be near 1
    {
        slotsPerDimension = pow(vec.size(),1/(3.));
        if(slotsPerDimension<2) {slotsPerDimension=2;}	// 8 cells is the minimum possible
    }

    // First, find numeric limits
    //initialize limits using three vectors of two doubles (min and max)
    limits	= vector< vector<double> >();

    limits.push_back(vector<double>());
    limits.push_back(vector<double>());
    limits.push_back(vector<double>());

    if(vec.size()<1)
    {
        throw("GridTree::GridTree smalish vector???? ");
    }


    // initialize maximum and minimum values to the coordinates of the first point
    limits[0].push_back(vec[0]->getX()); //minim en X
    limits[0].push_back(vec[0]->getX()); //minim en X
    limits[1].push_back(vec[0]->getY()); //minim en X
    limits[1].push_back(vec[0]->getY()); //minim en X
    limits[2].push_back(vec[0]->getZ()); //minim en X
    limits[2].push_back(vec[0]->getZ()); //minim en X


    // traverse grid updating limits
    vector<myPoint*>::iterator it;
    for(it = vec.begin(); it != vec.end(); it++) {

        myPoint * currentP = (*it);

        if(currentP->getX() < limits[0][0]) limits[0][0] = currentP->getX();
        if(currentP->getX() > limits[0][1]) limits[0][1] = currentP->getX();

        if(currentP->getY() < limits[1][0]) limits[1][0] = currentP->getY();
        if(currentP->getY() > limits[1][1]) limits[1][1] = currentP->getY();

        if(currentP->getZ() < limits[2][0]) limits[2][0] = currentP->getZ();
        if(currentP->getZ() > limits[2][1]) limits[2][1] = currentP->getZ();
    }


    // now that we know its dimensions, distribute the points throughout the grid
    // memory initialization
    grid = vector<vector<vector<Cell *> > >(slotsPerDimension);

    for(int i=0;i<slotsPerDimension;i++)
    {
        grid[i] = vector<vector<Cell *> >(slotsPerDimension);
        for(int j=0;j<slotsPerDimension;j++)
        {
            grid[i][j]=vector<Cell *>(slotsPerDimension);
            for (int k = 0; k < slotsPerDimension; ++k) {
                grid[i][j][k] = new Cell();
            }
        }
    }



    for(it=vec.begin();it!=vec.end();it++)
    {
        int x,y,z;
        myPoint * currentP = (*it);

        x = findSlot(currentP->getX(),'x');
        y = findSlot(currentP->getY(),'y');
        z = findSlot(currentP->getZ(),'z');

        grid[x][y][z]->addPoint(currentP);
    }

    kdtreezation();
}

GridTree::~GridTree() {

    for (int i = 0; i < slotsPerDimension; ++i) {
        for (int j = 0; j < slotsPerDimension; ++j) {
            for (int k = 0; k < slotsPerDimension; ++k) {

                delete grid[i][j][k];
            }
        }
    }
}

void GridTree::kdtreezation(){

//    int thsPoints = 5;

    for (int i = 0; i < slotsPerDimension; ++i) {
        for (int j = 0; j < slotsPerDimension; ++j) {
            for (int k = 0; k < slotsPerDimension; ++k) {

                Cell *cell = grid[i][j][k];

                cell->kdtreezation(thsPoints);
            }
        }
    }

}

int GridTree::findSlot(double val, char type,bool margin)
{
    double min,max;
    int returnValue;

    //cout<<"GridTree::findSlot limits "<<endl<<"x: ("<<limits[0][0]<<" , "<<limits[0][1]<<")"<<endl;
    //cout<<"y: ("<<limits[1][0]<<" , "<<limits[1][1]<<")"<<endl;
    //cout<<"z: ("<<limits[2][0]<<" , "<<limits[2][1]<<")"<<endl;

    switch( type )
    {
        case 'x' :
            min = limits[0][0];
            max = limits[0][1];
            break;
        case 'y' :
            min = limits[1][0];
            max = limits[1][1];
            break;
        case 'z' :
            min = limits[2][0];
            max = limits[2][1];
            break;

        default  : cout<<"GridTree::findSlot(double val, char type) wrong slot type???? "<<endl;
            throw("GridTree::findSlot(double val, char type) wrong slot type???? ");
            break;
    }


    //check for extreme cases 
    if(fabs(max-val)<tol) returnValue=slotsPerDimension-1;
    else if(fabs(min-val)<tol) returnValue=0;
    else
    {
        double pas = (fabs(max-min)/slotsPerDimension);

        returnValue = (int)( (val-min) /pas);
    }

    if( (returnValue>=slotsPerDimension) || (returnValue<0) )
    {
        if(!margin)
        {
            cout<<"GridTree::findSlot(double val, char type) wrong slot? "<<returnValue<<endl;
            throw("GridTree::findSlot(double val, char type) wrong slot? ");
        }
        else 	// set to the last slot out-of-bound queries (for example, for sentinel-guided searches
        {
            if(returnValue>=slotsPerDimension) returnValue=slotsPerDimension-1;
            else returnValue=0;
        }
    }

    //cout<<"GridTree::findSlot finished "<<returnValue<<endl<<endl<<endl<<endl;

    return returnValue ;

}



// return the minimum and maximum index of the slots affected
vector<int> GridTree::slotsTouched(double min, double max, char type)
{
    vector<int> returnValue = vector<int>(2);

    returnValue[0] = findSlot(min, type,true);
    returnValue[1] = findSlot(max, type,true);

    return returnValue;
}

vector<myPoint *> GridTree::neighbors(myPoint *p, double eps)
{

    //cout<<"GridTree::neigbors neighbors search for "<<p<<" at distance "<<eps<<endl;
    // find points in a query cube and then choose the ones inside the query sphere
    vector<myPoint *> returnValue;

    vector<int> limitsX = slotsTouched(p->getX()-eps, p->getX()+eps, 'x');
    vector<int> limitsY = slotsTouched(p->getY()-eps, p->getY()+eps, 'y');
    vector<int> limitsZ = slotsTouched(p->getZ()-eps, p->getZ()+eps, 'z');

    //cout<<"GridTree::neigbors limits values found: "<<endl;
    //cout<<"x: ("<<limitsX[0]<<" , "<<limitsX[1]<<")"<<endl;
    //cout<<"y: ("<<limitsY[0]<<" , "<<limitsY[1]<<")"<<endl;
    //cout<<"z: ("<<limitsZ[0]<<" , "<<limitsZ[1]<<")"<<endl;


    for(int i=limitsX[0];i<=limitsX[1];i++)
    {
        for(int j=limitsY[0];j<=limitsY[1];j++)
        {
            for(int k=limitsZ[0];k<=limitsZ[1];k++)
            {
                Cell *currentCell = grid[i][j][k];

                if (currentCell->isKdtreezed()){

                    ANNpoint q;
                    q = annAllocPt(3);
                    q[0] = p->getX();
                    q[1] = p->getY();
                    q[2] = p->getZ();

                    ANNidxArray nnIdx = new ANNidx[2];
                    ANNdistArray dists = new ANNdist[2];

                    currentCell->getKdtree()->annkSearch(q, 2, nnIdx, dists, 0.0001);

                    myPoint *currentP = currentCell->getPoint(nnIdx[0]);
                    double dist = currentP->dist(*p);

                    if(dist <= eps) {
                        if (*p != *currentP) {
                            returnValue.push_back(currentCell->getPoint(nnIdx[0]));
                        }
                        else {
                            returnValue.push_back(currentCell->getPoint(nnIdx[1]));
                        }
                    }

                    delete[] nnIdx;
                    delete[] dists;
                    annDeallocPt(q);
                }
                else {
                    for (int i_p = 0; i_p < grid[i][j][k]->get_nPoints(); ++i_p) {
                        myPoint *currentP = currentCell->getPoint(i_p);
                        double dist = currentP->dist(*p);

                        if (*p != *currentP && dist <= eps) {
                            returnValue.push_back(currentP);
                        }
                    }
                }
            }
        }
    }

    return returnValue;
}


int GridTree::getNumElems() {

    return nPoints;
}

int GridTree::getSlotsPerDimension() {

    return slotsPerDimension;
}

float GridTree::getMeanHeight() {

    float sum_depth = 0;
    int nKdtreezed = 0;

    for (int i = 0; i < slotsPerDimension; ++i) {
        for (int j = 0; j < slotsPerDimension; ++j) {
            for (int k = 0; k < slotsPerDimension; ++k) {

                if(grid[i][j][k]->isKdtreezed()) {
                    nKdtreezed++;
                    ANNkdStats st;
                    grid[i][j][k]->getKdtree()->getStats(st);

                    sum_depth += st.depth;
                }
            }
        }
    }

    return sum_depth/nKdtreezed;
}
