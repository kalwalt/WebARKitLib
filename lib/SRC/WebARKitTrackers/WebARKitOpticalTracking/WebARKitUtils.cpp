#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <emscripten/emscripten.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h>

#define N   10

bool homographyValid(Mat H) {
    const double det = H.at<double>(0,0)*H.at<double>(1,1)-H.at<double>(1,0)*H.at<double>(0,1);
    return 1/N < fabs(det) && fabs(det) < N;
}