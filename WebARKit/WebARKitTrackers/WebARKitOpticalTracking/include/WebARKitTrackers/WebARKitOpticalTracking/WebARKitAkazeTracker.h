#ifndef WEBARKIT_AKAZE_TRACKER_H
#define WEBARKIT_AKAZE_TRACKER_H

#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>

namespace webarkit {

class WebARKitAkazeTracker : public WebARKitTracker {
  friend class WebARKitTracker;

  cv::Ptr<cv::AKAZE> akaze;
  cv::Ptr<cv::BFMatcher> matcher;

  cv::Mat refGray, refDescr;
  std::vector<cv::KeyPoint> refKeyPts;

public:
  WebARKitAkazeTracker();
  void initialize_gray_raw(uchar *refData, size_t refCols,
                           size_t refRows) override;
  void processFrameData(uchar *frameData, size_t frameCols, size_t frameRows,
                        ColorSpace colorSpace) override;

private:
  bool resetTracking(cv::Mat &frameCurr) override;
  void processFrame(cv::Mat &frame) override;
};

} // namespace webarkit

#endif
