//
// Created by Ferran Roure on 21/10/2015.
//

#include "det_NormalSpaceSampling.h"

det_NormalSpaceSampling::det_NormalSpaceSampling() {

    data = NULL;
}

det_NormalSpaceSampling::~det_NormalSpaceSampling() {

}

void det_NormalSpaceSampling::setData(Data *d) {

    data = d;
}

void det_NormalSpaceSampling::execute() {

//    extractNormalSpaceSampling(data->A, data->params.nSamples);
    extractNormalSpaceSampling(data->B, data->params.nSamples);
}

void det_NormalSpaceSampling::extractNormalSpaceSampling(ElementSet *X, int sample) {

    cout << "NormalSpaceSampling running..." << endl;

    AdapterCGAL acgal;
    list<PointVectorPair> points = acgal.points2CGAL_list(X->getAllpoints());

    // Estimates normals direction.
    // Note: pca_estimate_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    const int nb_neighbors = 3; // K-nearest neighbors = 3 rings
    CGAL::pca_estimate_normals(points.begin(), points.end(),
                               CGAL::First_of_pair_property_map<PointVectorPair>(),
                               CGAL::Second_of_pair_property_map<PointVectorPair>(),
                               nb_neighbors);

    // THIS OPTIMIZATION, IN LINUX PRODUCES A BOOST ERROR. IS A BUG ON BOOST 1.54. I TRIED TO SOLVED BUT I CAN'T.
//    // Orients normals.
//    // Note: mst_orient_normals() requires an iterator over points
//    // as well as property maps to access each point's position and normal.
//    std::list<PointVectorPair>::iterator unoriented_points_begin =
//            CGAL::mst_orient_normals(points.begin(), points.end(),
//                                     CGAL::First_of_pair_property_map<PointVectorPair>(),
//                                     CGAL::Second_of_pair_property_map<PointVectorPair>(),
//                                     nb_neighbors);


    X->setNormals(acgal.CGAL2normals(points));

    X->createFileFromData("../models/test4Scaled.ply", "../models/provanormals.ply", true);


    // Our base vector used to find the angles is (1,0,0)


}

