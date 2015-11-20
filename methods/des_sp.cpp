#include "des_sp.h"



des_SP::des_SP()
{
    data = NULL;
}

des_SP::~des_SP()
{}

/* SET DATA -------------------------------------------------------------
 *
 * This method sets Data variable.
 */
void des_SP::setData(Data *d){

    data = d;
}


void des_SP::execute(){

    if(data->params.realData) {
        calcDescriptors(data->A);
    }
    calcDescriptors(data->B);

}

void des_SP::calcDescriptors(ElementSet *X) {

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


    PointCloud< Histogram<153> >::Ptr descSP (new PointCloud< Histogram<153> >);

	// Setup spin image computation
    SpinImageEstimation<PointXYZ, Normal, Histogram<153> > describer(8, 0.5, 16);
    describer.setInputCloud (cloud);
    describer.setInputNormals (normals);
//    describer.setSearchMethod (kdtree);
    describer.setRadiusSearch (radiusSearch);
    // describer.setRadiusSearch (0.2);

  	// Actually compute the spin images
    describer.compute (*descSP);

    for (int i=0; i<descSP->size(); i++){

        Histogram<153> desc = descSP->at(i);

        vector<float> hist;

        for (int j=0; j<153; j++){

            hist.push_back(desc.histogram[j]);
        }

        DescHistogram *myDesc = new DescHistogram(hist);

        X->setPointDescriptor(i, myDesc);
    }

}
