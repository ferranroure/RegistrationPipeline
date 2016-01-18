//
// Created by ferran on 26/11/15.
//

#include "det_KinectSuperSampling.h"

det_KinectSuperSampling::det_KinectSuperSampling(){

}

det_KinectSuperSampling::~det_KinectSuperSampling(){

}

void det_KinectSuperSampling::setData(Data *d) {

    data = d;
}

void det_KinectSuperSampling::execute() {

    superSampling(data->A);
    superSampling(data->B);
}

 /* ----------------------------------------------------------------------------------------------------------------
  * KinectSuperSampling
  * Kinect's resolution is clearly defined and points are orginized in "depth lines". At long distances,
  * the resolution decresses. We gonna create new points between original lines of points in order to having more
  * resolution. We take two consecutive lines and, for each point in the first line, we find its nearest neighbour in
  * the next one. Then, a middle point is created between them.
  */
void det_KinectSuperSampling::superSampling(ElementSet *X) {

    cout << "[ Kinect Super Sampling method ]" << endl;

    // Separate points by depth value
    int zmin = X->getZmin();
    int zmax = X->getZmax();
    vector< vector<Point*> > OrgPoints(zmax - zmin + 1);

    // Organize points by its Z coordinate. The location in the vector is given by its Z coordinate
    // minus zmin, in order to locate the nearest point at position 0. Each vector with points with same Z are called
    // "lines"
    for (int i = 0; i < X->nPoints(); ++i) {

        Point *p = X->getPoint(i);

        OrgPoints.at(p->getZ() - zmin).push_back(p);
    }

    vector<Point*> newPoints;

    // Create new point between lines
    // For each line of points
    for (int i1 = 0; i1 < OrgPoints.size()-1; ++i1) {

        // it exits points at this line
        if(!OrgPoints[i1].empty()){

            // Find the next line to create a new line in the middle.
            int i2 = i1 + 1;
            while(OrgPoints[i2].empty() && i2 < OrgPoints.size()) i2++;

            // Ktree of the first line
            myKdtree kdtree(&(OrgPoints[i1]));
            // For each point in the next line
            for (int m = 0; m < OrgPoints[i2].size(); ++m) {

                // Find the nearest neighbor of p2 in the first line
                Point *p2 = OrgPoints[i2][m];
                returnData rd = kdtree.calcOneNN(p2);
                Point *p1 = OrgPoints[i1][rd.index];

                // Create a new point in the middle of both points from both lines.
                Point *newP = new Point( (p1->getX()+p2->getX())/2,
                                         (p1->getY()+p2->getY())/2,
                                         (p1->getZ()+p2->getZ())/2);
                newP->setColor((p1->getRed()+p2->getRed())/2,
                               (p1->getGreen()+p2->getGreen())/2,
                               (p1->getBlue()+p2->getBlue())/2,
                                0);
                newPoints.push_back(newP);
            }
        }
    }

    int oldP = X->nPoints();

    // Add new points in the original cloud.
    for (int j = 0; j < newPoints.size(); ++j) {

        X->addPoint(newPoints[j]);
    }
    X->update();

    X->createFileFromData("../../models/resampled.ply", false, true);
    cout << X->nPoints()-oldP << " point have been added." << endl;

}
