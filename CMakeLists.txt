# CMake compilation instructions for Pipeline Project
#
# Required Libraries:
#	- C++ (sudo apt-get install build-essentials)
#	- Cmake (sudo apt-get install cmake)
#	- ANN lib (compile and copy to: /usr/local/lib/)
#   - Libnabo -> (compile and install (maybe libann.so is needed (apt get libann-dev))
#	- Point Cloud Libraries (pcl) sudo apt-get install libpcl-dev
#   - OpenCV sudo apt-get install libopencv-dev
#	- VTK libraries (5.10 or superior) https://thilinasameera.wordpress.com/2013/02/09/setting-up-vtk-and-itk-environments-for-ubuntu/
#   - Install CGAL Library -> sudo apt-get install libcgal-dev

# sudo ln -s  /usr/lib/x86_64-linux-gnu/libproj.so.<your-version> /usr/lib/x86_64-linux-gnu/libproj.so
#
#
# For MAC: (using homebrew)
#   - Install PCL (includes eigen and vtk) -> brew install pcl --without-apps --without-tools --without-qt --HEAD
#   - Install openCV -> brew install opencv
#   - Install ANN -> brew install ann
#   - Install CGAL Library -> brew install cgal
#
#
# Author: Ferran Roure (froure@eia.udg.edu)
# Date: 13/12/2013


project( Pipeline )

cmake_minimum_required(VERSION 2.6.2)
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6)
    if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3)
        cmake_policy(VERSION 2.8.4)
    else()
        cmake_policy(VERSION 2.6)
    endif()
endif()

#cmake_policy(SET CMP0022 NEW)

# --> Check required packages

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(CGAL REQUIRED)
if(CGAL_FOUND)
        message(--  CGAL found!)
        include_directories(${CGAL_INCLUDE_DIRS})
        link_directories(${CGAL_LIBRARY_DIRS})
endif()

FIND_LIBRARY (  ANN_LIB
        NAMES ANN
        PATHS /usr/local/lib/ann_1.1.2/lib )

include_directories( /usr/local/lib/ann_1.1.2/include)
link_directories(/usr/local/lib/ann_1.1.2/lib)



FIND_LIBRARY (  NEWMAT_LIB
        NAMES newmat
        PATHS external/newmat10 )



find_package(PCL 1.2 REQUIRED)
if ( PCL_FOUND )
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()


find_package( OpenCV REQUIRED)

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})

if ( ANN_LIB AND PCL_FOUND AND NEWMAT_LIB AND VTK_FOUND AND CGAL_FOUND )

    message( STATUS "All required libraries found!" )


    # Sources
    SET( SOURCES
            main.cpp
            data.cpp
            methods/det_iss.cpp
            methods/det_RandomSampling.cpp
            methods/det_NormalSpaceSampling.cpp
            methods/det_ColorSpaceSampling.cpp
            methods/det_DistConnectedComponents.cpp
            methods/det_KinectSuperSampling.cpp
            methods/des_shot.cpp
            methods/des_sp.cpp
            inputhandler.cpp
            outputhandler.cpp
            pipeline.cpp
            methods/ref_icp.cpp
            methods/ss_SmartForce.cpp
            methods/ss_4PCS.cpp
            methods/ss_3PS.cpp
            methods/ss_Grid3D.cpp
            include/elementset.cpp
            include/myicp.cpp
            include/DataStructures/kdtree/mykdtree.cpp
            include/DataStructures/octree/myOctree.cpp
            include/DataStructures/trihash/myTriHash.cpp
            include/DataStructures/compressedOctree/myCompressedOctree.cpp
            include/DataStructures/noDataStructure/noDataStructure.cpp
            include/DataStructures/gridTree/myGridTree.cpp
#            include/DataStructures/flann/myFlann.cpp
            include/DataStructures/kdtreeCV/myKdtreeCV.cpp
            include/mypcl.cpp
            include/plyio.cpp
            include/point.cpp
            include/timer.cpp
            include/descHistogram.cpp
            include/base.cpp
            include/descdist.cpp
            include/Converters/converterPCL.cpp
            include/Converters/converter4PCS.cpp
            include/Converters/converterCGAL.cpp
            include/Converters/converterYago.cpp
            include/Converters/converterKdtree.cpp
            include/Converters/converterGridTree.cpp
#            include/Converters/converterFlann.cpp
            include/Converters/converterKdtreeCV.cpp
            include/4pcsRegistrationObject.cpp
            include/DataStructures/octree/Octree.cpp
            include/DataStructures/octree/Node.cpp



            external/vector3D.cpp
            external/motion3D.cpp
            external/FineMatching/fineMatching.cpp
            external/FineMatching/reorder_verts.cc
            external/FineMatching/remove.cc
            external/FineMatching/overlap.cc
#            external/FineMatching/mesh_align.cc
            external/FineMatching/fineMatching.cpp
            external/FineMatching/filter.cc
            external/FineMatching/TriMesh_tstrips.cc
            external/FineMatching/TriMesh_stats.cc
            external/FineMatching/TriMesh_pointareas.cc
            external/FineMatching/TriMesh_normals.cc
            external/FineMatching/TriMesh_io.cc
            external/FineMatching/TriMesh_grid.cc
            external/FineMatching/TriMesh_connectivity.cc
            external/FineMatching/TriMesh_bounding.cc
            external/FineMatching/KDtree.cc
            external/FineMatching/ICP.cc
            external/newmat10/svd.cpp
            external/newmat10/submat.cpp
            external/newmat10/sort.cpp
            external/newmat10/solution.cpp
            external/newmat10/newmatrm.cpp
            external/newmat10/newmatnl.cpp
            external/newmat10/newmatex.cpp
            external/newmat10/newmat9.cpp
            external/newmat10/newmat8.cpp
            external/newmat10/newmat7.cpp
            external/newmat10/newmat6.cpp
            external/newmat10/newmat5.cpp
            external/newmat10/newmat4.cpp
            external/newmat10/newmat3.cpp
            external/newmat10/newmat2.cpp
            external/newmat10/newmat1.cpp
            external/newmat10/newfft.cpp
            external/newmat10/myexcept.cpp
            external/newmat10/jacobi.cpp
            external/newmat10/hholder.cpp
            external/newmat10/fft.cpp
            external/newmat10/evalue.cpp
            external/newmat10/cholesky.cpp
            external/newmat10/bandmat.cpp
            external/tinyxml2.cpp
            external/MoreStructures/CompressedOctree/CompressedOctree.cpp
            external/MoreStructures/CompressedOctree/CompressedONode.cpp
            external/MoreStructures/CompressedOctree/CompressedInformacioGeometrica.cpp
            external/MoreStructures/CompressedOctree/CompressedCandidateZone.cpp
            external/MoreStructures/TriHash/TriHash.cpp
            external/MoreStructures/AuxiliaryClasses/Element.cpp
            external/MoreStructures/AuxiliaryClasses/point3D.cpp
            external/MoreStructures/GridTree/GridTree.cpp
            external/MoreStructures/GridTree/Cell.cpp
            external/MoreStructures/GridTree/myPoint.cpp
            include/IDescriptor.h
            include/IManagerIO.h
            include/DataStructures/IDataStructure.h

            IDetection.h
            IDescription.h
            ISearchingStrategy.h
            ISearchingStrategy.cpp
            IRefinement.h )

    # -pg -> profiler
    # -frounding-math -> something for CGAL library
#    set(CMAKE_CXX_FLAGS "-std=c++11 -frounding-math")
    #    set(CMAKE_CXX_FLAGS "-pg -frounding-math")
    set(CMAKE_CXX_FLAGS "-frounding-math")

    add_executable( Pipeline ${SOURCES} )
    target_link_libraries(Pipeline ${ANN_LIB} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${VTK_LIBRARIES} ${CGAL_LIB} -L/opt/lib)

    #${NEWMAT_LIB} -> sembla que no cal.

else()

    # Show an error message for each of the missing libraries
    if ( NOT ANN_LIB )
        message( STATUS "ANN Libraries NOT found!" )
    endif()

endif()
