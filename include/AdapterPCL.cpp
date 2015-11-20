#include "AdapterPCL.h"
#include "mypcl.h"

AdapterPCL::AdapterPCL()
{
}

AdapterPCL::~AdapterPCL(){

}

PointCloud<PointXYZ>::Ptr AdapterPCL::points2PCL(vector<Point*> *points){


    PointCloud<PointXYZ>::Ptr cloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);

    for(int i=0; i<points->size(); i++){

        cloud->push_back(PointXYZ(points->at(i)->getX(), points->at(i)->getY(), points->at(i)->getZ()));
    }

    vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

    return cloud;
}


vector<int> AdapterPCL::points2PCLindices(vector<Point *> *points) {

    vector<int> indices;

    for (int i = 0; i < points->size(); ++i) {

        indices.push_back(points->at(i)->getIndex());
    }

    return indices;
}

vector<Point> *AdapterPCL::PCL2points(PointCloud<PointXYZ>::Ptr cloud) {
    return NULL;
}

PointCloud<Normal>::Ptr AdapterPCL::calcNormals(PointCloud<PointXYZ>::Ptr cloud, float radiusSearch) {

    PointCloud<Normal>::Ptr normals;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud (cloud);
    //normal_estimation.setViewPoint(FLT_MAX, FLT_MAX, FLT_MAX);
    //normal_estimation.setViewPoint(0,0,0);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod (kdtree);

    normals = pcl::PointCloud<pcl::Normal>::Ptr( new pcl::PointCloud< pcl::Normal> );
    normal_estimation.setRadiusSearch (radiusSearch);
//    normal_estimation.setKSearch(30);
//    normal_estimation.setRadiusSearch (0.03);
    normal_estimation.compute (*normals);
    /*
    float vpx, vpy, vpz;
    normal_estimation.getViewPoint(vpx, vpy, vpz);
    repairNormalsPCL(normals, vpx, vpy, vpz);
*/
    repairNormals(cloud, normals);

    return normals;
}


void AdapterPCL::repairNormals(PointCloud<PointXYZ>::Ptr workpoints, PointCloud<pcl::Normal>::Ptr normals) {

    Eigen::Vector4f c;
    int n = pcl::compute3DCentroid(*workpoints, c);


    Point centroid(c[0], c[1], c[2]);

    int nFlip = 0;

    for(int i=0; i<workpoints->size(); i++){

        vector3D N(normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z);
        Point p(workpoints->at(i).x, workpoints->at(i).y, workpoints->at(i).z);
        // d = -ax0 -by0 -cz0;
        float d = -(N.getX()*p.getX()) - (N.getY()*p.getY()) - ((N.getZ()*p.getZ()));

        float dist = ( (N.getX()*centroid.getX()) + (N.getY()*centroid.getY()) + (N.getZ()*centroid.getZ()) + d );// / N.modulus() ;

        if(dist > 0){
            nFlip++;
            normals->at(i).normal_x = (normals->at(i).normal_x)*(-1);
            normals->at(i).normal_y = (normals->at(i).normal_y)*(-1);
            normals->at(i).normal_z = (normals->at(i).normal_z)*(-1);
        }
    }
    //cout << "HE FLIPAT " << nFlip << " VEGADES I TINC " << normals->size() << " NORMALS" <<  endl;
}


PointCloud<SHOT352>::Ptr AdapterPCL::desc2SHOT(vector<Point*> * points){

    PointCloud<SHOT352>::Ptr desc = PointCloud<SHOT352>::Ptr(new PointCloud<SHOT352>);

    int descSize = points->at(0)->getDescSize();

    for(int i=0; i<points->size(); i++){

        SHOT352 d;
        for (int j = 0; j < descSize; ++j){

            d.descriptor[j] = points->at(i)->getDescriptor()->getValue(j);
        }

        desc->push_back(d);
    }

//    vector<int> indices;
//    pcl::removeNaNFromPointCloud(*desc,*desc, indices);


    return desc;
}

PointCloud<FPFHSignature33>::Ptr AdapterPCL::desc2FPFH(vector<Point*> * points){

    PointCloud<FPFHSignature33>::Ptr desc = PointCloud<FPFHSignature33>::Ptr(new PointCloud<FPFHSignature33>);

    int descSize = points->at(0)->getDescSize();

    for(int i=0; i<points->size(); i++){
        for (int j = 0; j < descSize; ++j){

            desc->at(i).histogram[j] = points->at(i)->getDescriptor()->getValue(j);
        }
    }

//    vector<int> indices;
//    pcl::removeNaNFromPointCloud(*desc,*desc, indices);

    return desc;
}

PointCloud<ShapeContext1980>::Ptr AdapterPCL::desc23DSC(vector<Point*> * points){

    PointCloud<ShapeContext1980>::Ptr desc = PointCloud<ShapeContext1980>::Ptr(new PointCloud<ShapeContext1980>);

    int descSize = points->at(0)->getDescSize();

    for(int i=0; i<points->size(); i++){
        for (int j = 0; j < descSize; ++j){

            desc->at(i).descriptor[j] = points->at(i)->getDescriptor()->getValue(j);
        }
    }

//    vector<int> indices;
//    pcl::removeNaNFromPointCloud(*desc,*desc, indices);

    return desc;
}

PointCloud< Histogram<153> >::Ptr AdapterPCL::desc2SpinImage(vector<Point*> * points){

    PointCloud< Histogram<153> >::Ptr desc = PointCloud< Histogram<153> >::Ptr(new PointCloud< Histogram<153> >);

    int descSize = points->at(0)->getDescSize();

    for(int i=0; i<points->size(); i++){
        for (int j = 0; j < descSize; ++j){

            desc->at(i).histogram[j] = points->at(i)->getDescriptor()->getValue(j);
        }
    }

//    vector<int> indices;
//    pcl::removeNaNFromPointCloud(*desc,*desc, indices);

    return desc;
}

