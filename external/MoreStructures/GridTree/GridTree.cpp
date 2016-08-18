#include "GridTree.h"

GridTree::GridTree(vector<myPoint*> &vec, int numC, int _thrsKdtree)
{
    slotsPerDimension = numC;
    tol=TOLERANCE;
    thrsKdtree = _thrsKdtree;

    nPoints = vec.size();

    if(slotsPerDimension==-1) // in the case where the number of cells was not specified, make charge factor to be near 1
    {
        slotsPerDimension = pow(vec.size(),1/(3.));
//        slotsPerDimension = 30;
        if(slotsPerDimension<2) {slotsPerDimension=2;}	// 8 cells is the minimum possible
    }

//    cout << slotsPerDimension << ";";

    // First, find numeric limits
    //initialize limits using three vectors of two doubles (min a4nd max)
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


    // set also slot sizes
    slotSizeX=fabs(limits[0][1]-limits[0][0])/slotsPerDimension;
    slotSizeY=fabs(limits[1][1]-limits[1][0])/slotsPerDimension;
    slotSizeZ=fabs(limits[2][1]-limits[2][0])/slotsPerDimension;

    sqSlotSizeX=slotSizeX*slotSizeX;
    sqSlotSizeY=slotSizeY*slotSizeY;
    sqSlotSizeZ=slotSizeZ*slotSizeZ;

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
                grid[i][j][k] = new Cell(limits[0][0]+i*slotSizeX,limits[0][0]+(i+1)*slotSizeX,limits[1][0]+j*slotSizeY,limits[1][0]+(j+1)*slotSizeY,limits[2][0]+k*slotSizeZ,limits[2][0]+(k+1)*slotSizeZ);
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

    calcMeanPoints();

//    cout << "slots: " << pow(slotsPerDimension,3) << " NeC: " << notEmptyCells
//         << " %: " <<  (100*notEmptyCells)/pow(slotsPerDimension,3) << " meanPoints: " << meanPoints << endl;

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

void GridTree::calcMeanPoints(){

    int sum = 0;
    int sum_i = 0;

    for (int i = 0; i < slotsPerDimension; ++i) {
        for (int j = 0; j < slotsPerDimension; ++j) {
            for (int k = 0; k < slotsPerDimension; ++k) {

                if(! grid[i][j][k]->empty()){

                    sum += grid[i][j][k]->get_nPoints();
                    sum_i++;
                }
            }
        }
    }

    meanPoints = sum / sum_i;
    notEmptyCells = sum_i;
}

void GridTree::kdtreezation(){

    for (int i = 0; i < slotsPerDimension; ++i) {
        for (int j = 0; j < slotsPerDimension; ++j) {
            for (int k = 0; k < slotsPerDimension; ++k) {

                Cell *cell = grid[i][j][k];

                cell->kdtreezation(thrsKdtree*(meanPoints/5));
//                cell->kdtreezation(thrsKdtree);
            }
        }
    }

}

int GridTree::findSlot(double val, char type,bool margin, bool squared)
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

    if(squared){
        min = (min<0) ? min*min*-1 : min*min;
        max = (max<0) ? max*max*-1 : max*max;
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
vector<int> GridTree::slotsTouched(double min, double max, char type, bool squared)
{
    vector<int> returnValue = vector<int>(2);

    returnValue[0] = findSlot(min, type,true, squared);
    returnValue[1] = findSlot(max, type,true, squared);

    return returnValue;
}


vector<myPoint *> GridTree::neighbors(myPoint *p, double eps)
{

    //cout<<"GridTree::neigbors neighbors search for "<<p<<" at distance "<<eps<<endl;
    // find points in a query cube and then choose the ones inside the query sphere
    vector<myPoint *> returnValue;
    double sqrEps = eps * eps;

    vector<int> limitsX = slotsTouched(p->getX()-eps, p->getX()+eps, 'x');
    vector<int> limitsY = slotsTouched(p->getY()-eps, p->getY()+eps, 'y');
    vector<int> limitsZ = slotsTouched(p->getZ()-eps, p->getZ()+eps, 'z');


//    if(p->getIndex() == 0){
////
//        cout<<"GridTree::neigbors limits values found: "<<endl;
//        cout<<"x: ("<<limitsX[0]<<" , "<<limitsX[1]<<")"<<endl;
//        cout<<"y: ("<<limitsY[0]<<" , "<<limitsY[1]<<")"<<endl;
//        cout<<"z: ("<<limitsZ[0]<<" , "<<limitsZ[1]<<")"<<endl;
//    }

    Super4PCS::KdTree<double>::VectorType qP;
    qP << p->getX(),
            p->getY(),
            p->getZ();


    for(int i=limitsX[0];i<=limitsX[1];i++)
    {
        for(int j=limitsY[0];j<=limitsY[1];j++)
        {
            for(int k=limitsZ[0];k<=limitsZ[1];k++)
            {
                Cell *currentCell = grid[i][j][k];


                if (currentCell->isKdtreezed()){

                    vector<int> indices;

                    currentCell->getKdtree()->doQueryDistIndices(qP, sqrEps, indices);

                    if(!indices.empty()) {

                        for(int i=0; i<indices.size(); i++){

                            returnValue.push_back(currentCell->getPoint(indices[i]));
                        }
                    }

                }
                else {
                    for (int i_p = 0; i_p < grid[i][j][k]->get_nPoints(); ++i_p) {
                        myPoint *currentP = currentCell->getPoint(i_p);
                        double sqrDist = currentP->sqrdist(*p);
                        if(sqrDist <= sqrEps) {
                            if (*p != *currentP) {
                                returnValue.push_back(currentP);
                            }
                        }
                    }
                }
            }
        }
    }

    return returnValue;
}


myPoint * GridTree::oneNeighbor(myPoint *p, double eps)
{

    myPoint * NN = NULL;
    double sqrEps = eps * eps;
    double bestSqrDist = DBL_MAX;

    // First, we gonna search a NN in the falling slot. If we find one, we use the distance
    // between p and its NN as eps.
    // find the falling slot for p
    int slotx = findSlot(p->getX(), 'x', true);
    int sloty = findSlot(p->getY(), 'y', true);
    int slotz = findSlot(p->getZ(), 'z', true);

    Cell *currentCell = grid[slotx][sloty][slotz];

    if (currentCell->isKdtreezed()){
        Super4PCS::KdTree<double>::VectorType qP;
        qP << p->getX(),
                p->getY(),
                p->getZ();

        // sqrDist will be updated inside doQueryRestrictedClosestIndex() function
        // with the distance between qP and its NN.
        double sqrDist = sqrEps;
        Super4PCS::KdTree<double>::Index resId = currentCell->getKdtree()->doQueryRestrictedClosestIndex(qP, sqrDist);

        if(resId != Super4PCS::KdTree<double>::invalidIndex()) {

            // AIXO COMPROVA SI SON DIFERENTS O NO
           /* double nari = p->sqrdist(*(currentCell->getPoint(resId)));
            if(fabs(sqrDist-nari)>0.000000000000001)
            {
                cout << sqrDist << " " << nari << endl;
                cout<<"MASSA DIFERENTS           *******************************************************************:"<<endl;
                exit(-1);
          }
            sqrDist = nari;*/

            if (sqrDist <= sqrEps && sqrDist < bestSqrDist) {

                NN = currentCell->getPoint(resId);
                bestSqrDist = sqrDist;
                sqrEps = bestSqrDist;
            }
        }
    }
    else {

        for (int i_p = 0; i_p < currentCell->get_nPoints(); ++i_p) {

            myPoint *currentP = currentCell->getPoint(i_p);
            double sqrDist = currentP->sqrdist(*p);

            if(sqrDist <= sqrEps && sqrDist < bestSqrDist) {
                if (*p != *currentP) {
                    NN = currentP;
                    bestSqrDist = sqrDist;
                    sqrEps = bestSqrDist;
                }
            }
        }
    }

    vector<int> limitsX;
    vector<int> limitsY;
    vector<int> limitsZ;


    // Rewrite this part if we can't use sqrDist!
    if(NN != NULL){
        sqrEps = bestSqrDist;
        bool touchOut=currentCell->touchOut(p->getX(),p->getY(),p->getZ(),sqrEps);
        if(touchOut) {
            eps = sqrt(bestSqrDist);

            limitsX = slotsTouched(p->getX() - eps, p->getX() + eps, 'x', false);
            limitsY = slotsTouched(p->getY() - eps, p->getY() + eps, 'y', false);
            limitsZ = slotsTouched(p->getZ() - eps, p->getZ() + eps, 'z', false);
        }
        else{
            //do nothing
            limitsX = vector<int>(2);
            limitsY = vector<int>(2);
            limitsZ = vector<int>(2);
            limitsX[0]=slotx;
            limitsX[1]=slotx;
            limitsY[0]=sloty;
            limitsY[1]=sloty;
            limitsZ[0]=slotz;
            limitsZ[1]=slotz;

        }
/*        if(!touchOut&& ((limitsX[0]!=slotx )||(limitsX[1]!=slotx )||(limitsY[0]!=sloty )||(limitsY[1]!=sloty )||(limitsZ[0]!=slotz )||(limitsZ[1]!=slotz ) ))
        {

            cout<<" ************************************************************************************************************************************ostiaPuta "<< p->getX()<<endl;
            exit(-1);

        }*/


        // alternative way,x
      /*  double lowQueue=p->getX()-currentCell->getXMin();
        lowQueue=(lowQueue*lowQueue)-sqrEps;

        int otherLimitsXLow;
        int offset1;
        if(lowQueue>0) {
            offset1 = ceil(lowQueue / sqSlotSizeX);
            otherLimitsXLow = slotx - offset1;
        }
        else{otherLimitsXLow = slotx;}

        double highQueue=currentCell->getXMax()-p->getX();
        highQueue=(highQueue*highQueue)-sqrEps;

        int otherLimitsXHigh;
        int offset2;
        if(highQueue>0) {
            int offset2 = ceil(highQueue / sqSlotSizeX);
            otherLimitsXHigh = slotx + offset2;
        }
        else{otherLimitsXHigh = slotx;}


        if(otherLimitsXLow<0) {otherLimitsXLow=0;}
        if(otherLimitsXHigh>limits[0][1]) {otherLimitsXHigh=limits[0][1];}

        //check the alternative way is right
        if (otherLimitsXLow != limitsX[0]) {
            cout<<" p->getX() "<< p->getX()<<endl;
            cout<<" current cell "<< currentCell->getXMin()<<" "<<currentCell->getXMax()<<endl;
            cout<<" lowQueue "<< lowQueue <<" "<<lowQueue/sqSlotSizeX<<endl;
            cout<<" epsilon "<< eps<<endl;
            cout<<" epsilnSq "<< sqrEps<<endl;
            cout<<" slots jumped "<<offset1<<" "<<offset2 <<endl;


            cout << "11111111111111111111111111111111111111111111111111111111Cagada x al quadrat " << otherLimitsXLow << " hauria de ser " << limitsX[0] << endl;
            exit(-1);
        }
        if (otherLimitsXHigh != limitsX[1]) {
            cout << "22222222222222222222222222222222222222222222222222222222222222Cagada x al quadrat alt " << otherLimitsXHigh << " hauria de ser " << limitsX[1] << endl;
            exit(-1);
        }*/

    }
    else{

        limitsX = slotsTouched(p->getX()-eps, p->getX()+eps, 'x', false);
        limitsY = slotsTouched(p->getY()-eps, p->getY()+eps, 'y', false);
        limitsZ = slotsTouched(p->getZ()-eps, p->getZ()+eps, 'z', false);
    }



    for(int i=limitsX[0];i<=limitsX[1];i++)
    {
        for(int j=limitsY[0];j<=limitsY[1];j++)
        {
            for(int k=limitsZ[0];k<=limitsZ[1];k++)
            {
                if(i==slotx && j==sloty && k==slotz) continue;

                Cell *currentCell = grid[i][j][k];

                if(!currentCell->xPossiblytouched(p->getX(),sqrEps)||!currentCell->yPossiblytouched(p->getY(),sqrEps)||!currentCell->zPossiblytouched(p->getZ(),sqrEps)) continue;

                if (currentCell->isKdtreezed()){

                    Super4PCS::KdTree<double>::VectorType qP;
                    qP << p->getX(),
                            p->getY(),
                            p->getZ();

                    // sqrDist will be updated inside doQueryRestrictedClosestIndex() function
                    // with the distance between qP and its NN.
                    double sqrDist = sqrEps;
                    Super4PCS::KdTree<double>::Index resId = currentCell->getKdtree()->doQueryRestrictedClosestIndex(qP, sqrDist);

                    if(resId != Super4PCS::KdTree<double>::invalidIndex()) {

                        // AIXO COMPROVA SI SON DIFERENTS O NO
                      //  double nari = p->sqrdist(*(currentCell->getPoint(resId)));
//            if(sqrDist!=nari) cout << sqrDist << " " << nari << endl;
                 //       sqrDist = nari;

                /*        double nari = p->sqrdist(*(currentCell->getPoint(resId)));
                        if(fabs(sqrDist-nari)>0.000000000000001)
                        {
                            cout << sqrDist << " " << nari << endl;
                            cout<<"2MASSA DIFERENTS           *******************************************************************:"<<endl;
                           // exit(-1);
                        }
                        sqrDist = nari;*/

                        if (sqrDist <= sqrEps && sqrDist < bestSqrDist) {

                            NN = currentCell->getPoint(resId);
                            bestSqrDist = sqrDist;
                            sqrEps = bestSqrDist;
                        }
                    }
                }
                else {
                    int nPointsInCell=grid[i][j][k]->get_nPoints();
                    for (int i_p = 0; i_p <nPointsInCell ; ++i_p) {
                        myPoint *currentP = currentCell->getPoint(i_p);
                        double sqrDist = currentP->sqrdist(*p);

                        if(sqrDist <= sqrEps && sqrDist < bestSqrDist) {
                            if (*p != *currentP) {

                                NN = currentP;
                                bestSqrDist = sqrDist;
                                sqrEps = bestSqrDist;
                            }
                        }
                    }
                }
            }
        }
    }

    return NN;
}

int GridTree::getNumElems() {

    return nPoints;
}

int GridTree::getSlotsPerDimension() {

    return slotsPerDimension;
}

float GridTree::getMeanHeight() {

//    float sum_depth = 0;
//    int nKdtreezed = 0;
//
//    for (int i = 0; i < slotsPerDimension; ++i) {
//        for (int j = 0; j < slotsPerDimension; ++j) {
//            for (int k = 0; k < slotsPerDimension; ++k) {
//
//                if(grid[i][j][k]->isKdtreezed()) {
//                    nKdtreezed++;
//                    ANNkdStats st;
//                    grid[i][j][k]->getKdtree()->getStats(st);
//
//                    sum_depth += st.depth;
//                }
//            }
//        }
//    }
//
//    return sum_depth/nKdtreezed;
}

int GridTree::numberSlotsTouchedSquared(double sqEps, char type)
{
    switch( type )
    {
        case 'x' :
            return sqEps/sqSlotSizeX;
            break;
        case 'y' :
            return sqEps/sqSlotSizeY;
            break;
        case 'z' :
            return sqEps/sqSlotSizeZ;
            break;

        default  : cout<<"GridTree::numberSlotsTouchedSquared wrong slot type???? "<<endl;
            throw("GridTree::numberSlotsTouchedSquared wrong slot type????  ");
            break;
    }
    return 0;
}
