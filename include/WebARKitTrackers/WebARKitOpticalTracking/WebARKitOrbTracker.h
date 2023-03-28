#ifndef WEBARKIT_ORB_TRACKER_H
#define WEBARKIT_ORB_TRACKER_H

#include <emscripten.h>
#include <emscripten/val.h>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>


using namespace emscripten;

class WebARKitOrbTracker {
  cv::Ptr<cv::ORB> orb;
  cv::Ptr<cv::BFMatcher> matcher;

  cv::Mat refGray, refDescr;
  std::vector<cv::KeyPoint> refKeyPts;

  cv::Mat H;
  std::vector<cv::Point2f> corners;

  cv::Mat prevIm;
  int numMatches;
  std::vector<cv::Point2f> framePts;
  std::vector<double> output; // 9 from homography matrix, 8 from warped corners*/
public:
  WebARKitOrbTracker();
  void initialize_gray_raw(unsigned char *refData, size_t refCols, size_t refRows);
  void processFrameData(unsigned char *frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace);
  std::vector<double> getOutputData();
  bool isValid();
  emscripten::val getCorners();

private:
  bool resetTracking(cv::Mat frameCurr);
  bool track(cv::Mat frameCurr);
  void processFrame(cv::Mat frame);
  bool homographyValid(cv::Mat H);
  void fill_output(cv::Mat H);
  void clear_output();
  bool _valid;
  bool initialized;
};
#endif
