#ifndef WEBARKIT_ORB_TRACKER_H
#define WEBARKIT_ORB_TRACKER_H

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <emscripten.h>
#include <emscripten/val.h>

using namespace emscripten;

class WebARKitOrbTracker {
    bool initialized = false;
    bool _valid = false;

    cv::Ptr<cv::ORB> orb = NULL;
    cv::Ptr<cv::BFMatcher> matcher = NULL;

    cv::Mat refGray, refDescr;
    std::vector<cv::KeyPoint> refKeyPts;

    cv::Mat H;
    std::vector<cv::Point2f> corners;

    cv::Mat framePrev;
    int numMatches = 0;
    std::vector<cv::Point2f> framePts;
    double *output; // 9 from homography matrix, 8 from warped corners*/
public:
    WebARKitOrbTracker();
    void initialize(unsigned char * refData, size_t refCols, size_t refRows);
    void processFrameData(unsigned char * frameData, size_t frameCols, size_t frameRows);
    double * getOutputData();
private:
    bool resetTracking(cv::Mat frameCurr);
    bool track(cv::Mat frameCurr);
    void processFrame(cv::Mat frame);
    bool homographyValid(cv::Mat H);
    void fill_output(cv::Mat H, double *output);
    void clear_output();
};
#endif
