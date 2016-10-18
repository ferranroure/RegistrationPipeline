#include "ss_Super4PCS.h"

ss_Super4PCS::ss_Super4PCS(){

    data = NULL;
}

ss_Super4PCS::~ss_Super4PCS() {

}

void ss_Super4PCS::setData(Data *d) {

    data = d;
}

void ss_Super4PCS::execute() {

    converterSuper4PCS cs4pcs;
    vector<match_4pcs::Point3D> set1 = cs4pcs.convertArray(data->A->getWorkpoints(), true, true);
    vector<match_4pcs::Point3D> set2 = cs4pcs.convertArray(data->B->getWorkpoints(), true, true);

    // Our matcher.
    match_4pcs::Match4PCSOptions options;
   // time Super4PCS -i bust0.obj bust1.obj -o 0.9 -d 8 -t 1000 -r bust-output.obj
    // Set parameters.
    cv::Mat mat = cv::Mat::eye(4, 4, CV_64F);
    options.overlap_estimation = 0.8;
//    options.sample_size = 210;
//    options.max_normal_difference = 360;
//    options.max_color_distance = 0;
    options.max_time_seconds = 10000;
    options.delta = 6;
    // Match and return the score (estimated overlap or the LCP).
    float score = 0;

    match_4pcs::MatchSuper4PCS matcher(options);

    Timer timer;
    timer.reset();
    score = matcher.ComputeTransformation(set1, &set2, &mat);
    double time = timer.elapsed();
    cout << "Computing time: " << time << endl;
    cout << "Score: " << score << endl;
    data->cM = cs4pcs.convertMatrix(mat);
}
