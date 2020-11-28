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

class WebARKitOrbTracker {
public:
    void initialize(std::string filename, size_t refCols, size_t refRows);
    double *resetTracking(uchar frameData[], size_t frameCols, size_t frameRows);
    double *track(uchar frameData[], size_t frameCols, size_t frameRows);
private:
    bool initialized = false;

    cv::Ptr<cv::ORB> orb = NULL;
    cv::Ptr<cv::BFMatcher> matcher = NULL;

    cv::Mat refGray, refDescr;
    std::vector<cv::KeyPoint> refKeyPts;

    cv::Mat H;
    std::vector<cv::Point2f> corners;

    cv::Mat framePrev;
    int numMatches = 0;
    std::vector<cv::Point2f> framePts;
    double *output; // 9 from homography matrix, 8 from warped corners
    cv::Mat im_gray(uchar data[], size_t cols, size_t rows);
    bool homographyValid(cv::Mat H);
    void fill_output(cv::Mat H);
    void clear_output();
};
#endif
