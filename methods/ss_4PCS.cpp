#include "ss_4PCS.h"

ss_4PCS::ss_4PCS(){

    data = NULL;
}

ss_4PCS::~ss_4PCS() {

}

void ss_4PCS::setData(Data *d) {

    data = d;
}

void ss_4PCS::execute() {


    // Bunny: 0.1, 500, 0.1, 0.3
    // Buddha: 0.2 / 0.1, 1000, 0.1, 0.2
    // Joints: 0.2, 1000, 0.1, 0.2
    // Bust: 0.1, 1000, 0.1, 0.2
    // Frog: 0.2, 1000, 0.1, 0.3


    // Aquest thr seria % de punts aparellats mínims a partir del qual considerem que s'ha fet registre. Si és -1 s'agafa l'estimació d'overlap "overlap".
    float thr = 0.2; // THRESHOLD FOR THE bestf

    // Número de punts que consideren. Més o menys, pq en fan un tractament random raro.
    int n_points = 1000;

    // Diferències entre les normals. No sé perquè ho fan servir.
    float norm_diff = 360; //30

    // This parameter is the multiplication factor applied to the threshold for corresponding searching.
    // p = q  if( dist(p,q) < (meanDistOfAllPoints * 2 ) * delta = eps; also called eps.
    float delta = 0.1;

    // This parameter is used to select the 4th point in de base from A. Is provided in order to not select a point
    // which its correspondence in Q falls in a non-overlapping area.
    // qd = diam*overlap*2.0; -> this qd is used to find a wide point. length(u)< qd.
    float overlap = 0.5;

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

    Timer timer;
    timer.reset();
    float a = matcher.compute(*set1, *set2, delta, overlap, mat);
    double time = timer.elapsed();
    cout << "Computing Time: " << time << endl;

    data->cM = a4pcs.mat2motion(mat);
//    data->cM->write(cout);
    delete set1, set2;
}
