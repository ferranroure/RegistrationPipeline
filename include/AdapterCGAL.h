//
// Created by Ferran Roure on 26/10/2015.
//

#ifndef ADAPTERCGAL_H
#define ADAPTERCGAL_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <list>
#include <utility> // defines std::pair


#include "point.h"
#include <vector>

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point_3, Vector_3> PointVectorPair;


class AdapterCGAL {

public:

    AdapterCGAL();
    ~AdapterCGAL();

    list<PointVectorPair> points2CGAL_list(vector<Point *> *cloud);
    vector<vector3D> * CGAL2normals(list<PointVectorPair> points);

};


#endif //ADAPTERCGAL_H
