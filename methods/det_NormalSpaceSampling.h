//
// Created by Ferran Roure on 21/10/2015.
//

#ifndef DET_NORMALSPACESAMPLING_H
#define DET_NORMALSPACESAMPLING_H

#include "../IDetection.h"
#include "../include/Converters/converterCGAL.h"
#include "../include/point.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <utility> // defines std::pair
#include <list>

#include "../include/plyio.h"


class det_NormalSpaceSampling : public IDetection{

public:
    det_NormalSpaceSampling();
    ~det_NormalSpaceSampling();

    void setData(Data *d);
    void execute();

    void extractNormalSpaceSampling(ElementSet *X, int sample);
};


#endif //DET_NORMALSPACESAMPLING_H
