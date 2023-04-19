#ifndef WEBARKIT_ORB_TRACKER_H
#define WEBARKIT_ORB_TRACKER_H

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
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>

#ifdef __EMSCRIPTEN__
using namespace emscripten;
#endif

class WebARKitOrbTracker : public WebARKitTracker
{

  friend class WebARKitTracker;

  cv::Ptr<cv::ORB> orb;
  cv::Ptr<cv::BFMatcher> matcher;

  cv::Mat refGray, refDescr;
  std::vector<cv::KeyPoint> refKeyPts;

  cv::Mat H;
  std::vector<cv::Point2f> corners;

  cv::Mat prevIm;
  int numMatches;
  std::vector<cv::Point2f> framePts;

public:
  WebARKitOrbTracker();
  void initialize_gray_raw(unsigned char *refData, size_t refCols, size_t refRows) override;
  void processFrameData(unsigned char *frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace) override;

private:
  bool resetTracking(cv::Mat frameCurr) override;
  bool track(cv::Mat frameCurr) override;
  void processFrame(cv::Mat frame) override;
  bool initialized;
};
#endif
