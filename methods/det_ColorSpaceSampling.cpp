//
// Created by Ferran Roure on 21/10/2015.
//

#include "det_ColorSpaceSampling.h"

det_ColorSpaceSampling::det_ColorSpaceSampling() {

    data = NULL;
}

det_ColorSpaceSampling::~det_ColorSpaceSampling() {

}

void det_ColorSpaceSampling::setData(Data *d) {

    data = d;
}

void det_ColorSpaceSampling::execute() {

//    extractColorSpaceSampling(data->A, data->params.nSamples);
//    extractColorSpaceSampling(data->B, data->params.nSamples);

//    data->A->createFileFromData("../models/provaColorsA.ply", false, true);
//    data->B->createFileFromData("../models/provaColorsB.ply", false, true);

}

void det_ColorSpaceSampling::extractColorSpaceSampling(ElementSet *X, int sample) {

    cout << "ColorSpaceSampling running..." << endl;

    // Divisions of the angles.
    int div = 100;
    float dv = 360/div; // divisor

    // Create 2D array to store the pointColor vectors for sampling
    typedef boost::multi_array< vector<Point*>, 2> array2d;
    typedef array2d::index index;
    array2d binTable(boost::extents[div][div]);

    // NORMALIZATION IS NEEDED!! USE VECTOR3D
    for(vector<Point*>::iterator it = X->getAllpoints()->begin(); it!= X->getAllpoints()->end(); ++it){

        float r = ((float)(*it)->getRed())/255;
        float g = ((float)(*it)->getGreen())/255;
        float b = ((float)(*it)->getBlue())/255;

        vector3D colorVec(r,g,b);
        //colorVec.normalize();

        // find spherical polar coordinates.
        float alpha = atan(colorVec.y/colorVec.x);
        float beta  = acos(colorVec.z); // it should be divided by radius, but is 1.

        // getting row and column to store the Point.
        int row = (alpha*(180/M_PI))/dv;
        int col = (beta*(180/M_PI))/dv;
//        cout << colorVec << " " << alpha*(180/M_PI) << " " << beta << " " << row << " " << col << endl;

        binTable[row][col].push_back(*it);

    }

    X->getWorkpoints()->clear();

    int nGets = 1000; // gets per cell;
    int s1 = X->allpoints->size()/nGets;
    srand(1962);

    for(index i = 0; i < div; ++i){
        for(index j = 0; j < div; ++j) {

            int k = 0;
            int count = 0;

            // get all points in the cell
            if (binTable[i][j].size() <= nGets) {

                for (int k = 0; k < binTable[i][j].size(); ++k) {

                    X->getWorkpoints()->push_back(binTable[i][j].at(k));
                }
            }
            // get nGets points randomly.
            else {
                while (count < nGets) {
                    if (rand() % s1 == 0) {
                        X->getWorkpoints()->push_back(binTable[i][j].at(k));
                        count++;
                    }
                }
            }


            // select the first of each bin. It seems like a border detector.
//            for(int k=0; k<500; ++k){
//
//                if(k<binTable[i][j].size()) {
//                    X->getWorkpoints()->push_back(binTable[i][j].at(k));
//                }
//                else continue;
//
//            }
//            cout << binTable[i][j].size() << " ";
        }
//        cout << endl;
    }

    cout << "Size: " << X->getWorkpoints()->size() << endl;

    cout << "Color Space sampling done. All ok" << endl;

//    X->createFileFromData("../models/provaColors.ply", false, true);

//    exit(0);













//    // Assign values to the elements
//    int values = 0;
//    for(index i = 0; i != 3; ++i)
//        for(index j = 0; j != 4; ++j)
//            for(index k = 0; k != 2; ++k)
//                A[i][j][k] = values++;







//    X->setNormals(acgal.CGAL2normals(points));
//
//    X->createFileFromData("../models/test4Scaled.ply", "../models/provanormals.ply", true);


    // Our base vector used to find the angles is (1,0,0)


}