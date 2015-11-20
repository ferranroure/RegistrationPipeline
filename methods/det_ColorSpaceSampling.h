//
// Created by Ferran Roure on 21/10/2015.
//

#ifndef DET_ColorSpaceSampling_H
#define DET_ColorSpaceSampling_H

#include "../IDetection.h"
#include "../include/AdapterCGAL.h"
#include "../include/point.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <utility> // defines std::pair
#include <list>
#include "boost/multi_array.hpp"

#include "../include/plyio.h"


class det_ColorSpaceSampling : public IDetection{

public:
    det_ColorSpaceSampling();
    ~det_ColorSpaceSampling();

    void setData(Data *d);
    void execute();

    void extractColorSpaceSampling(ElementSet *X, int sample);
};


#endif //DET_ColorSpaceSampling_H
