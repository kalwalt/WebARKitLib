#ifndef WEBARKIT_TRACKER_H
#define WEBARKIT_TRACKER_H

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/val.h>
#endif
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "WebARKitEnums.h"

#ifdef __EMSCRIPTEN__
using namespace emscripten;
#endif

class WebARKitTracker
{
public:
    WebARKitTracker();

    virtual void initialize_gray_raw(unsigned char *refData, size_t refCols, size_t refRows) = 0;
    virtual void processFrameData(unsigned char *frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace) = 0;
    std::vector<double> getOutputData();
    bool isValid();

#ifdef __EMSCRIPTEN__
    emscripten::val getCorners();
#endif

protected:
    virtual bool resetTracking(cv::Mat frameCurr) = 0;
    virtual bool track(cv::Mat frameCurr) = 0;
    virtual void processFrame(cv::Mat frame) = 0;
    bool homographyValid(cv::Mat H);
    void fill_output(cv::Mat H);
    void clear_output();
    bool _valid;

private:
    std::vector<cv::Point2f> corners;
    std::vector<double> output; // 9 from homography matrix, 8 from warped corners*/
};

#endif