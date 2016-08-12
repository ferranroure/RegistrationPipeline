#include "converterPCL.h"

converterPCL::converterPCL()
{
}

converterPCL::~converterPCL(){

}

PointCloud<PointXYZ>::Ptr converterPCL::points2PCL(vector<Point*> *points){


    PointCloud<PointXYZ>::Ptr cloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);


    for(int i=0; i<points->size(); i++){

        cloud->push_back(PointXYZ(points->at(i)->getX(), points->at(i)->getY(), points->at(i)->getZ()));
    }


    vector<int> indices;
    removeNaNFromPointCloud(*cloud,*cloud, indices);



    return cloud;
}


vector<int> converterPCL::points2PCLindices(vector<Point *> *points) {

    vector<int> indices;

    for (int i = 0; i < points->size(); ++i) {

        indices.push_back(points->at(i)->getIndex());
    }

    return indices;
}

vector<Point> *converterPCL::PCL2points(PointCloud<PointXYZ>::Ptr cloud) {

    vector<Point> *pointsToReturn = new vector<Point>();

    PointCloud<PointXYZ>::iterator it;

    for (it = cloud->points.begin(); it < cloud->points.end(); it++) {

        pointsToReturn->push_back(Point(it->x,it->y,it->z));

    }

    return pointsToReturn;

}

PointCloud<Normal>::Ptr converterPCL::calcNormals(PointCloud<PointXYZ>::Ptr cloud, float radiusSearch) {

   PointCloud<Normal>::Ptr normals;

    NormalEstimation<PointXYZ, Normal> normal_estimation;
    normal_estimation.setInputCloud (cloud);

    search::KdTree<PointXYZ>::Ptr kdtree (new search::KdTree<PointXYZ>);
    normal_estimation.setSearchMethod (kdtree);

    normals = PointCloud<Normal>::Ptr( new PointCloud< Normal> );
    normal_estimation.setRadiusSearch (radiusSearch);
    normal_estimation.compute (*normals);

    repairNormals(cloud, normals);

    return normals;


/*
    PointCloud<Normal>::Ptr normals;
    normals.reset(new pcl::PointCloud<pcl::Normal> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
    normal_estimation_filter.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation_filter.setSearchMethod (search_tree);

    normal_estimation_filter.setRadiusSearch (radiusSearch);
    normal_estimation_filter.compute (*normals);

    return normals;

*/

}


void converterPCL::repairNormals(PointCloud<PointXYZ>::Ptr workpoints, PointCloud<Normal>::Ptr normals) {

    Eigen::Vector4f c;
    int n = compute3DCentroid(*workpoints, c);


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


PointCloud<SHOT352>::Ptr converterPCL::desc2SHOT(vector<Point*> * points){

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
//    removeNaNFromPointCloud(*desc,*desc, indices);


    return desc;
}

PointCloud<FPFHSignature33>::Ptr converterPCL::desc2FPFH(vector<Point*> * points){

   // cout<<"converterPCL::desc2FPFH start "<<points->size()<<endl;


    PointCloud<FPFHSignature33>::Ptr desc = PointCloud<FPFHSignature33>::Ptr(new PointCloud<FPFHSignature33>);

    int descSize = points->at(0)->getDescSize();

//    cout<<"converterPCL::desc2FPFH looping "<<points->size()<<" with size "<<descSize<<endl;

    for(int i=0; i<points->size(); i++){
//        cout<<"converterPCL::desc2FPFH looping "<<i<<"/"<<points->size()<<endl;

        FPFHSignature33 d;
        for (int j = 0; j < descSize; ++j){
  //          cout<<"converterPCL::desc2FPFH looping inside "<<j<<"/"<<descSize<<endl;

            d.histogram[j] = points->at(i)->getDescriptor()->getValue(j);
        }
        desc->push_back(d);
    }

//    vector<int> indices;
//    removeNaNFromPointCloud(*desc,*desc, indices);

   // cout<<"converterPCL::desc2FPFH end "<<endl;


    return desc;
}

PointCloud<ShapeContext1980>::Ptr converterPCL::desc23DSC(vector<Point*> * points){

    PointCloud<ShapeContext1980>::Ptr desc = PointCloud<ShapeContext1980>::Ptr(new PointCloud<ShapeContext1980>);

    int descSize = points->at(0)->getDescSize();

    for(int i=0; i<points->size(); i++){
        ShapeContext1980 d;
        for (int j = 0; j < descSize; ++j){

            d.descriptor[j] = points->at(i)->getDescriptor()->getValue(j);
        }
        desc->push_back(d);
    }

//    vector<int> indices;
//    removeNaNFromPointCloud(*desc,*desc, indices);

    return desc;
}

PointCloud< Histogram<153> >::Ptr converterPCL::desc2SpinImage(vector<Point*> * points){

    PointCloud< Histogram<153> >::Ptr desc = PointCloud< Histogram<153> >::Ptr(new PointCloud< Histogram<153> >);

    int descSize = points->at(0)->getDescSize();

    for(int i=0; i<points->size(); i++){

        Histogram<153> d;

        for (int j = 0; j < descSize; ++j){

            d.histogram[j] = points->at(i)->getDescriptor()->getValue(j);
        }

        desc->push_back(d);

    }

//    vector<int> indices;
//    removeNaNFromPointCloud(*desc,*desc, indices);

    return desc;
}

