#include "ss_4PCS.h"

ss_4PCS::ss_4PCS(){
 //   cout<<"creator 4PCS "<<endl;
    // Parameter default values

    // Bunny: 0.1, 500, 0.1, 0.3
    // Buddha: 0.2 / 0.1, 1000, 0.1, 0.2
    // Joints: 0.2, 1000, 0.1, 0.2
    // Bust: 0.1, 1000, 0.1, 0.2
    thr = 0.2;
    n_points = 1000;
    norm_diff = 360;
    delta = 0.1;
    overlap = 0.4;

    data = NULL;
}

ss_4PCS::~ss_4PCS() {

}

void ss_4PCS::setData(Data *d) {
    //cout<<"ss_4PCS::setData "<<endl;

    data = d;

    // now check if we had received the parameters via the command line
    if(d->params.fourPUseCmdLineP)
    {
//        cout<<"ss_4PCS::setdata changing shit! "<<endl;

        thr=d->params.thr;                          // threshold for sets to be considered matched
        n_points=d->params.nPoints;                       // Number of points sampled at the start of the 4points algorithm
        norm_diff=d->params.normDiff;                    // difference between normals?
        delta=d->params.delta;                        // distance allowed for two points to be considered neighbors also works with base points
        overlap=d->params.overlap;
    }
}

double ss_4PCS::execute() {
 //   cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!ss_4PCS::execute() running 4points starting execute "<<endl;

    converter4PCS a4pcs;
    vector<Point3D> * set1 = a4pcs.points24PCS(data->A->getWorkpoints(), false, true);
    vector<Point3D> * set2 = a4pcs.points24PCS(data->B->getWorkpoints(), false, true);

    fpcsRegistrationObject matcher;
    double mat[4][4];
    matcher.setThreshold(thr);
    matcher.setNumberOfPoints(n_points);
    matcher.setNormDiff(norm_diff);
    matcher.setUseNormal(true);
    matcher.setDataStructType(data->params.dataStructure);

    cout<<"ss_4PCS::execute() running 4points data structure with the following parameters "<<thr<<" "<<n_points<<" "<<norm_diff<<" "<<delta<<" "<<overlap<<endl;

    float a = matcher.compute(*set1, *set2, delta, overlap, mat);
    data->cM = a4pcs.mat2motion(mat);
//    data->cM->write(cout);
    delete set1, set2;

    return a;
}
