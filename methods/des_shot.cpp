#include "des_shot.h"



des_SHOT::des_SHOT()
{
    data = NULL;
}

des_SHOT::~des_SHOT()
{}

/* SET DATA -------------------------------------------------------------
 *
 * This method sets Data variable.
 */
void des_SHOT::setData(Data *d){

    data = d;
}


void des_SHOT::execute(){

    if(data->params.realData) {
        calcDescriptors(data->A);
    }
    calcDescriptors(data->B);

}

void des_SHOT::calcDescriptors(ElementSet *X) {

    // Translate from our format to PCL.
    AdapterPCL apcl;
    PointCloud<PointXYZ>::Ptr cloud = apcl.points2PCL(X->getAllpoints());
//    PointCloud<PointXYZ>::Ptr surface = apcl.points2PCL(X->getAllpoints());
//    vector<int> indices = apcl.points2PCLindices(X->getWorkpoints());
//    boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));


    float radiusNormal = X->getMMD() * data->params.radiusNormalFactor;
    float radiusSearch = X->getMMD() * data->params.radiusSearchFactor;

    // Computing normals
//    PointCloud<Normal>::Ptr normals = apcl.calcNormals(surface, radiusNormal);
    PointCloud<Normal>::Ptr normals = apcl.calcNormals(cloud, radiusNormal);

    PointCloud<SHOT352>::Ptr descSHOT (new PointCloud<SHOT352>());
    SHOTEstimationOMP<PointXYZ, Normal, SHOT352> describer;

    describer.setRadiusSearch (radiusSearch);
    describer.setInputCloud (cloud);
    describer.setInputNormals (normals);
//    describer.setSearchSurface (surface);
//    describer.setIndices(indicesptr);   //AIXO NO XUTA I NO SÉ PERQUÈ.
    describer.compute (*descSHOT);

    for (int i=0; i<descSHOT->size(); i++){

        SHOT352 desc = descSHOT->at(i);

        vector<float> hist;

        for (int j=0; j<352; j++){

            hist.push_back(desc.descriptor[j]);
        }

        DescHistogram *myDesc = new DescHistogram(hist);

        X->setPointDescriptor(i, myDesc);
    }

}
