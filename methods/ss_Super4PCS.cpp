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

    // Our matcher.
    match_4pcs::Match4PCSOptions options;

    // Set parameters.
//    cv::Mat mat = cv::Mat::eye(4, 4, CV_64F);
//    options.overlap_estimation = overlap;
//    options.sample_size = n_points;
//    options.max_normal_difference = norm_diff;
//    options.max_color_distance = max_color;
//    options.max_time_seconds = max_time_seconds;
//    options.delta = delta;
    // Match and return the score (estimated overlap or the LCP).
    float score = 0;

    match_4pcs::MatchSuper4PCS matcher(options);

    cout << "caca" << endl;
}
