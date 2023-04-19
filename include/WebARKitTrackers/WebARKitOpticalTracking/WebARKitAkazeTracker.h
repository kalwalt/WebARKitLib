#ifndef WEBARKIT_AKAZE_TRACKER_H
#define WEBARKIT_AKAZE_TRACKER_H

#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>

class WebARKitAkazeTracker : public WebARKitTracker
{
  friend class WebARKitTracker;

  cv::Ptr<cv::AKAZE> akaze;
  cv::Ptr<cv::BFMatcher> matcher;

  cv::Mat refGray, refDescr;
  std::vector<cv::KeyPoint> refKeyPts;

  cv::Mat H;

  cv::Mat framePrev;
  int numMatches;
  std::vector<cv::Point2f> framePts;

public:
  WebARKitAkazeTracker();
  void initialize_gray_raw(unsigned char *refData, size_t refCols, size_t refRows) override;
  void processFrameData(unsigned char *frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace) override;

private:
  bool resetTracking(cv::Mat frameCurr) override;
  bool track(cv::Mat frameCurr) override;
  void processFrame(cv::Mat frame) override;
  bool initialized;
};
#endif
