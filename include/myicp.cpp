#include "myicp.h"

/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
myICP::myICP(){

    fm = NULL;
}


/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
myICP::myICP(ElementSet *A, ElementSet *B){

    TriMesh *mesh1 = new TriMesh();
    TriMesh *mesh2 = new TriMesh();

    // NOTE: "Point" is from our pipeline, "point" is from vec.h, from FineMatching algorithm.
    for(vector<Point>::iterator it=A->getPoints()->begin(); it!= A->getPoints()->end(); ++it){

        mesh1->vertices.push_back(point(it->getX(), it->getY(), it->getZ()));
    }

    for(vector<Point>::iterator it=B->getPoints()->begin(); it!= B->getPoints()->end(); ++it){

        mesh2->vertices.push_back(point(it->getX(), it->getY(), it->getZ()));

    }

    fm = new fineMatching(mesh1, mesh2);
}


/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
myICP::myICP(string infile1, string infile2){

    fm = new fineMatching(infile1.c_str(), infile2.c_str());
}


/* DESTRUCTOR -----------------------------------------------------------
 *
 */
myICP::~myICP(){

    delete fm;
}


/* ALIGN -----------------------------------------------------------
 *
 *  This method executes the ICP alignment method from external library.
 */
motion3D* myICP::align(){

    motion3D *motAfterICP = NULL;
    try{
        xform ix1 = xform();
        xform ix2 = xform();

        fm->iCP( ix1 , ix2 );

        motAfterICP = new motion3D(ix2);
    }
    catch (char const *c) {cerr<<"icp:: Exception after ICP!!!!!!!!!!!!!!!!!!!!!!! "<<endl;}

    return motAfterICP;
}
