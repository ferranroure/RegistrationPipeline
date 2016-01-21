#include "elementset.h"
#include "AdapterPCL.h"
#include "DataStructures/compressedOctree/myCompressedOctree.h"

/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
ElementSet::ElementSet(){

    //initRandomMachine();

    points = NULL;
    workpoints =  NULL;
    allpoints = NULL;
    dataStruct = NULL;
    octree = NULL;
    normals = NULL;
}


/* CONSTRUCTOR COPY ------------------------------------------------------
 *
 */
ElementSet::ElementSet(ElementSet &ES){

    //initRandomMachine();

    points = NULL;
    workpoints =  NULL;
    allpoints = NULL;
    dataStruct = NULL;
    octree = NULL;
    normals = NULL; // should be also copied!

    if(ES.points != NULL){

        points = new vector<Point>();

        for (vector<Point>::iterator it=ES.points->begin(); it!=ES.points->end(); it++){
            Point p(it->getX(), it->getY(), it->getZ());
            p.setIndex(it->getIndex());
            points->push_back(p);
        }

        createWorkingStructures();

        calcDiagonal();
        createDataStructure();
        calcMMD();

//        pcl = new myPCL(points, MMD);



    }
    else {points = NULL;}


}

void ElementSet::createWorkingStructures() {

    if(points != NULL){

        workpoints = new vector<Point*>();
        allpoints = new vector<Point*>();

        for (int i=0; i<points->size(); i++) {

            Point *p = &(points->at(i));
            p->setIndex(i);
            Point *q = &(points->at(i));
            q->setIndex(i);
            workpoints->push_back(p);
            allpoints->push_back(q);
        }
    }

}

/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
ElementSet::ElementSet(vector<Point> *lin){

    points = NULL;
    workpoints =  NULL;
    allpoints = NULL;
    dataStruct = NULL;
    octree = NULL;
    normals = NULL;

    //initRandomMachine();

    points = lin;

    createWorkingStructures();
    calcDiagonal();
    createDataStructure();
    calcMMD();

//    pcl = new myPCL(points, MMD);
}


/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
ElementSet::ElementSet(string file, float normFactor){

    //initRandomMachine();

    points = NULL;
    workpoints =  NULL;
    allpoints = NULL;
    dataStruct = NULL;
    octree = NULL;
    normals = NULL;


    PlyIO plyio;
    points = plyio.readFile(file);

    if(normFactor!=1){

        scalePoints(normFactor);
    }

    createWorkingStructures();
    calcDiagonal();
    createDataStructure();
    calcMMD();


//    pcl = new myPCL(points, MMD);
}


/* DESTRUCTOR -----------------------------------------------------------
 *
 */
ElementSet::~ElementSet(){


    if(dataStruct != NULL) delete dataStruct;
    if(allpoints != NULL) delete allpoints;
    if(workpoints != NULL) delete workpoints;
    if(points != NULL) delete points;
    if(octree != NULL) delete octree;
    if(normals != NULL) delete normals;
//    if(pcl != NULL) delete pcl;
}

/* Init Random Machine generator-----------------------------------------
 *
 *  Initializes the random number generator.
 */
void ElementSet::initRandomMachine(){

    srand (time(NULL));
    //srand (1);
}



/* UPDATE -----------------------------------------------------------
 *
 *  This method update the external information of this ElementSet.
 *  It must be executed after a tranformation.
 */
void ElementSet::update(){


    createDataStructure();
    calcMMD();

//    pcl->update(MMD);
}


/* CREATE FILE FROM DATA ---------------------------------------------------
 *
 * This method creates a ply file using information from an existing ply
 * file (header, faces...)  but putting our modified points from Data.
 */
//void ElementSet::createFileFromData(string infile, string outfile, bool withNormals){
//
//    PlyIO plyio;
//
//    if(withNormals == false) {
//        plyio.writeFile(infile, outfile, points);
//    }
//    else{
//        plyio.writeFile(infile, outfile, points, normals);
//    }
//}


// THIS IS THE GOOD METHOD, THE OTHER IS DEPRECATED
void ElementSet::createFileFromData(string outfile, bool withNormals, bool withColor) {

    PlyIO plyio;

    plyio.writeFile(outfile, workpoints, withNormals, withColor);
}


/* ADD NOISE ---------------------------------------------------------------
 *
 * This method adds synthetic noise to an object. We can control the level
 * of noise using the "threshold" parameter. "Threshold" provides the % of noise
 * aplied, related to the diagonal of the bounding box of the object
 */
void ElementSet::addNoise(double threshold){

    if(threshold != 0){

        // Generating random noise. For test, the seed is 1.
        //srand(time(NULL));
        double dist = getDiagonal();

        for(vector<Point>::iterator it=points->begin(); it!=points->end(); ++it)
        {
            // Calc random sign for the displacement.
            int sX = 1; int sY = 1; int sZ = 1;
            if (((double) rand() / (RAND_MAX))<0.5) sX = -1;
            if (((double) rand() / (RAND_MAX))<0.5) sY = -1;
            if (((double) rand() / (RAND_MAX))<0.5) sZ = -1;

            // Calc random value of the displacement.
            double nX = ((double) rand() / (RAND_MAX));
            double nY = ((double) rand() / (RAND_MAX));
            double nZ = ((double) rand() / (RAND_MAX));

            double m = sqrt(nX*nX + nY*nY + nZ*nZ);

            // random distance to move each component
            double rand_thrs = ((double) rand() / (RAND_MAX)) * threshold * MMD;

            // Adding the signed displacement to each point, with pondered threshold. (unitary vector)
            it->addNoise((nX*sX)/m, (nY*sY)/m, (nZ*sZ)/m, rand_thrs);
        }

        update();
    }
}



/* CALC DIAGONAL ------------------------------------------------------------------
 *
 *  This method calculates the maximum diagonal distance of the point cloud and
 *  vertexs of the bounding box. Also computes the approx center of mass.
 */
double ElementSet::calcDiagonal()
{
    xmax = -FLT_MAX;
    ymax = -FLT_MAX;
    zmax = -FLT_MAX;
    xmin = FLT_MAX;
    ymin = FLT_MAX;
    zmin = FLT_MAX;

    // Finding min point and max point.
    for(vector<Point>::iterator it=points->begin(); it!= points->end(); ++it)
    {

        if(it->getX() > xmax) xmax = it->getX();
        if(it->getY() > ymax) ymax = it->getY();
        if(it->getZ() > zmax) zmax = it->getZ();

        if(it->getX() < xmin) xmin = it->getX();
        if(it->getY() < ymin) ymin = it->getY();
        if(it->getZ() < zmin) zmin = it->getZ();
    }

    // Return distance between min and max.
    diagonal = sqrt(pow(xmax-xmin,2) + pow(ymax-ymin,2) + pow(zmax-zmin, 2));

    // Calc center
    center = Point( (xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2 );

    return diagonal;
}

Point ElementSet::calcCentroid(){

    calcDiagonal();
}


/* CALC MMD -----------------------------------------------------------
 *
 *  This method calculates the Mean Minimum Distance (MMD) between points
 *  of the point cloud.
 */
void ElementSet::calcMMD(){

    float sum = 0;

    for(vector<Point*>::iterator it=workpoints->begin(); it!=workpoints->end(); ++it){

        returnData rd = dataStruct->calcOwnNN(*it);

        sum += sqrt(rd.sqrDist);
//        cout << sqrt(rd.sqrDist) << endl;
    }

    MMD = sum / workpoints->size();


}


/* TRANSFORM ---------------------------------------------------------------------
 *
 *  This method aplies a transformation matrix to a vector of points.
 */
void ElementSet::transform(motion3D *m){

    if(m == NULL){
        cerr << "ElementSet::Tranformation failed! Motion is NULL!" << endl;
    }
    else{
        for(vector<Point>::iterator it=points->begin(); it!=points->end(); ++it){

            Point p = ((*m) * (*it));

            it->update(p);
        }
        update();
    }
}


/* CREATE DATA STRUCTURE ---------------------------------------------------------------------
 *
 *  This method calls the create method from DATASTRUCTURE class.
 */
void ElementSet::createDataStructure(){

    if(dataStruct!=NULL) delete dataStruct;

//    dataStruct = new myKdtree(workpoints);
//    dataStruct = new myOctree(workpoints);
//    dataStruct = new myTriHash(workpoints, diagonal);
    dataStruct = new myCompressedOctree(workpoints, diagonal);

    if(octree!=NULL) delete octree;
    //octree = new Octree(workpoints, 5, xmin, xmax, ymin, ymax, zmin, zmax);

}

/* CALC NN ---------------------------------------------------------------------
 *
 *  This method calls calcNN method from mykdtree class. Returns RMS distance
 *  Input:  - Q: Vector of query points.
 *          - nNn: Number of nearest neighbors detected for each query point.
 *          - threshold: Maximum distance to consider a point as a candidate of nearest neighbour. // NOSE SI CAL POSAR-LI!
 *          - percOfPoints: % of points of Q used as query points to get a RMS distance
 *  Output:  Root Mean Squeared distance of distances between query points and his neighbours.
 */
double ElementSet::calcNN(vector<Point> *Q, double percOfPoints, float errorFactor, int &pairedPoints){

    double MSD = 0;
    int err = 0;
    vector<int> NNv;

    //srand(time(NULL));
    double rn;

    for(vector<Point>::iterator it=Q->begin(); it!=Q->end(); ++it){

        rn = ((double) rand() / (RAND_MAX));
        if(rn <= percOfPoints){

            returnData rd = dataStruct->calcOneNN(&(*it));

            // counting errors
//            cout << sqrt(rd.sqrDist) << " " << MMD*errorFactor << endl;
            if(sqrt(rd.sqrDist) > MMD*errorFactor) err++;
            else {
                MSD += rd.sqrDist;
                NNv.push_back(rd.index);
            }
        }
    }

    cout << "err: " << err << endl;
    pairedPoints = NNv.size();

    double RMSD = sqrt(MSD/NNv.size()); // divided by number of valid points, not errors.

    return RMSD;
}

// Compute certain number of NN of a given point.
vector<returnData> ElementSet::calcNneigh(Point *q, int nNeigh) {

    return dataStruct->calcNneigh(q, nNeigh);

}


/* GET POINT -------------------------------------------------------------------
 *
 *  This method returns a pointer to an Point stored in the given position.
 */
Point * ElementSet::getPoint(int pos){

    Point *p = (workpoints->at(pos));
    return p;
}


/* GET RANDOM POINT -------------------------------------------------------------------
 *
 *  This method returns a pointer to a random point.
 */
Point * ElementSet::getRandomPoint(bool useDetectors){

    Point *p = NULL;

    if(useDetectors && !workpoints->empty()){

        int pos = rand() % workpoints->size();
        p = workpoints->at(pos);
//        p->setIndex(pos); // CANVIAR SI NO VOLEM FER SERVIR DETECTORS PER TOT EL CLOUD.
    }
    else{

        int pos = rand() % points->size();
        p = &(points->at(pos));
    }

    return p;
}

Base * ElementSet::getRandomBase(bool useDetectors){

    Point *i = getRandomPoint(useDetectors);
    Point *j = findWidePoint(i, useDetectors);
    Point *k = getRandomPoint(useDetectors);

//    i->print();
//    j->print();
//    k->print();


    return (new Base(i,j,k));
}


/* FIND WIDE POINT --------------------------------------------------------------
 *
 *  This method finds a partner point from an ElementSet which is located far
 *  (more than diagonal/2) from a certain point.
 */
Point * ElementSet::findWidePoint(Point *p, bool useDetectors){

    double minDist = (getDiagonal()) / 3;
    double dist = 0;
    Point *q = NULL;

    do{
        q = getRandomPoint(useDetectors);
        dist = p->dist(q);
    }while(dist < minDist);

    return q;
}


/* CALC DESCRIPTOR -------------------------------------------------------------------
 *
 *  Calculates point descriptors of this ElementSet.
 */
void ElementSet::calcDescriptor(string method, float radiusNormal, float radiusSearch){

    cerr << "ElementSet::Aquest mètoode calcDescriptors està deprecated!" << endl; exit(0);
//    pcl->calcDescriptors(method, radiusNormal, radiusSearch);
}



/* CALC DETECTOR -------------------------------------------------------------------
 *
 *  Calculates keypoints of this ElementSet.
 */
void ElementSet::calcDetector(string method){

    cerr << "ElementSet::Aquest mètoode calcDetectors està deprecated!" << endl; exit(0);
//
//    vector<Point> *keys = pcl->calcDetectors(method);
//
//    workpoints = new vector<Point*>;
//
//    // Identifying keypoints in the original model.
//
//    int j = 0;
//
//    for(vector<Point>::iterator it = keys->begin(); it!=keys->end(); ++it){
//
//        j++;
//        int i = 0;
//        while( i < points->size()){
//
//            if(*it == points->at(i)) break;
//            else i++;
//
//        }
//
//        if(i==points->size()){
//
//            cout << "No he trobat res" << endl;
//            it->print();
//        }
//        else{
//            workpoints->push_back(new Point(points->at(i)));
//        }
//
//    }
//
//    //cout << "Number of detected keypoints: " << keypoints->size() << endl;
}


Point * ElementSet::findPoint(Point &p){

    int i = 0;

    while(i < points->size()){

        if(p == points->at(i)){

//            cout << "he trobat un punt" << endl;
            return &points->at(i);
        }

        i++;
    }

    return NULL;
}

/* SORT POINTS -------------------------------------------------------------------
 *
 *  Sort the point list according its descriptor values.
 */
vector<DescDist> * ElementSet::sortPoints(Point *p){

    vector<DescDist> *sortedPoints = new vector<DescDist>();

    // add the pointers to points in the vector
    for(int i=0; i<workpoints->size(); i++){
        Point *q = workpoints->at(i);
//        cout << p->descriptor->getValue(0) << endl;
//        cout << q->descriptor->getValue(0) << endl << endl;
        sortedPoints->push_back(DescDist(q, p->getDescriptor()->compare(q->getDescriptor())));
    }

    sort(sortedPoints->begin(), sortedPoints->end());


    return sortedPoints;
}



/* NPOINTS -------------------------------------------------------------------
 *
 *  This method returns the number of points of this ElementSet.
 */
int ElementSet::nPoints(){

    return workpoints->size();
}


/* PRINT -----------------------------------------------------------------------
 *
 *  This method prints a vector of points.
 */
void ElementSet::print(bool withNormals, bool withColor){

    cout << "Printing vector of points:" << endl;
    cout << "------------------------" << endl;

    for(vector<Point>::iterator it=points->begin(); it!=points->end(); ++it)
    {
        it->print(withNormals, withColor);
    }

    cout << "-------------------------" << endl;
    cout << "vector of points is printed" << endl;
}


Eigen::Matrix3f * ElementSet::calcCovarianceMatrix(){

    cerr << "ElementSet::Aquest mètoode calcCovarianceMatrix està deprecated!" << endl; exit(0);
    return NULL;
//    return pcl->calcCovarianceMatrix();
}

void ElementSet::createFileFromDescriptors(string outfile){

    ofstream out (outfile.c_str());


    for(vector<Point>::iterator it=points->begin(); it!=points->end(); ++it){

        out << "ID: " << it->getIndex()
            << " Point: (" << it->getX() << "," << it->getY() << "," << it->getZ() << ") " << endl
            << (it->getDescriptor())->toString()
            << endl;
    }

    out.close();
}



bool ElementSet::availableDescriptors() {
    return points->at(0).availableDescriptor();
}

void ElementSet::setPointDescriptor(int pos, IDescriptor *desc) {

    points->at(pos).setDescriptor(desc);

}

// nCorr must be smaller thant nPoints();
vector<Point *> *ElementSet::findCorrespondences(Point *p, int nCorr, bool usePCL, string method) {

    vector<Point *> *ret = NULL;

    if(usePCL){

        myPCL pcl;
        ret = pcl.findCorrespondences(p, workpoints, nCorr, method);
    }
    else{


        vector<DescDist> *ddist = sortPoints(p);

        ret = new vector<Point *>();

        for (int i = 0; i < nCorr; ++i) {

            ret->push_back(ddist->at(i).pnt);
//            cout << ddist->at(i).id << " " << ddist->at(i).dist << endl;
        }

        delete ddist;
    }

    return ret;
}

void ElementSet::calcNormals(float radiusNormal) {

    // Translate from our format to PCL.
    AdapterPCL apcl;
    PointCloud<PointXYZ>::Ptr cloud = apcl.points2PCL(getWorkpoints());

    PointCloud<Normal>::Ptr normals = apcl.calcNormals(cloud, radiusNormal);

    for (int i = 0; i < normals->size(); ++i) {

        Normal norm = normals->at(i);

        getWorkpoints()->at(i)->setNormal(norm.normal_x, norm.normal_y, norm.normal_z);
        getWorkpoints()->at(i)->print(true);
    }

}

vector<pair<Point *, Point *> > ElementSet::findPairs(float distance, float thrs) {

    // Vars
    vector<pair<Point*, Point*> > ret;
//    myKdtree kdtree;

//    kdtree.create(workpoints);

    for (int i = 0; i < workpoints->size(); ++i) {

        Point *p = workpoints->at(i);

        for (int j = 0; j < workpoints->size(); ++j) {

            Point *q = workpoints->at(j);
//        returnData rd = kdtree.findPair(workpoints->at(pos), distance);

            if (abs(p->dist(q) - distance) <= thrs) {
//            cout << distance << " " << p->dist(q) << endl;
                pair<Point *, Point *> two(p, q);
                ret.push_back(two);
            }
        }
    }


    return ret;
}

// S'HA DE PASSAR EL THRS PER PARÀMETRE HOME! :D
vector<Point *> ElementSet::findCoplanarPoints(vector3D norm, float d, Point *p, float dist) {

    return octree->findCoplanarPoints(norm, d, p, dist);
}

vector<pair<Point *, Point *> > ElementSet::findPairsOctree(float distance, float thrs) {

    if(octree == NULL){

        cerr << "Octree == NULL. You have to create it! " << endl;
        exit(EXIT_FAILURE);
    }

    vector<pair<Point*, Point*> > res;

    for (int i = 0; i < workpoints->size(); ++i) {

        vector<pair<Point*, Point*> > temp = octree->findPairs(workpoints->at(i), distance, thrs);
        res.insert(res.end(), temp.begin(), temp.end());
    }

    return res;
}

int ElementSet::calcOutliers(float thrs) {

    //cout << "Looking for outliers at thrs = " << thrs << "........" << endl;
    int count = 0;

    for (int i = 0; i < allpoints->size(); ++i) {

        //returnData rd = dataStruct->calcOneNN(allpoints->at(i));
        vector<returnData> vrd = dataStruct->calcNneigh(allpoints->at(i), 10);

        for (int j = 0; j < vrd.size(); ++j) {

            if (vrd.at(j).sqrDist > thrs * thrs) {
                count++;
                break;
            }
        }
    }

    //cout << "I found " << count << " outliers" << endl;

    return count;
}

void ElementSet::scalePoints(float normFactor){


    for (vector<Point*>::iterator it = allpoints->begin(); it!=allpoints->end(); ++it) {

        (*it)->addCoords((*it)->getX()/normFactor,
                         (*it)->getY()/normFactor,
                         (*it)->getZ()/normFactor);
    }
}

void ElementSet::addPoint(Point *p) {

    points->push_back(*p);

}
