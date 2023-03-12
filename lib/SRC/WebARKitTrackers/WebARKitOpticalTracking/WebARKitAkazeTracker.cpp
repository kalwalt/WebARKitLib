#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitAkazeTracker.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h>


WebARKitAkazeTracker::WebARKitAkazeTracker()
    : corners(4), output(17, 0.0), _valid(false), initialized(false),
      akaze(nullptr), matcher(nullptr), numMatches(0) {}

void WebARKitAkazeTracker::initialize(unsigned char *refData, size_t refCols,
                                      size_t refRows) {
  std::cout << "Start!" << std::endl;
  std::cout << MAX_FEATURES << std::endl;
  akaze = cv::AKAZE::create();
  std::cout << "Akaze created!" << std::endl;
  matcher = cv::BFMatcher::create();
  std::cout << "BFMatcher created!" << std::endl;
  std::cout << "refCols: " << refCols << std::endl;
  std::cout << "refRows: " << refRows << std::endl;
  // cv::Mat refGray = im_gray(refData, refCols, refRows);
  cv::Mat colorFrame(refCols, refRows, CV_8UC3, refData);
  free(refData);
  cv::Mat refGray(refCols, refRows, CV_8UC1);
  cv::cvtColor(colorFrame, refGray, cv::COLOR_RGB2GRAY);
  std::cout << "Gray Image!" << std::endl;
  cv::Mat flippedImg;
  cv::flip(refGray, flippedImg, 1);
  std::cout << "Flipped Gray Image!" << std::endl;
  akaze->detectAndCompute(flippedImg, cv::noArray(), refKeyPts, refDescr);
  std::cout << "Reference image keypoints: " << refKeyPts.size() << std::endl;
  std::cout << "Reference image descriptors: " << refDescr.size() << std::endl;
  std::cout << "akaze Detect and Compute passed!" << std::endl;
  // std::cout << refDescr << std::endl;

  corners[0] = cvPoint(0, 0);
  corners[1] = cvPoint(refCols, 0);
  corners[2] = cvPoint(refCols, refRows);
  corners[3] = cvPoint(0, refRows);

  std::cout << "corners filled!" << std::endl;

  initialized = true;
  std::cout << initialized << std::endl;
  std::cout << "Ready!" << std::endl;
}

void WebARKitAkazeTracker::initialize_raw(unsigned char *refData,
                                          size_t refCols, size_t refRows) {
  std::cout << "Start!" << std::endl;
  std::cout << MAX_FEATURES << std::endl;
  akaze = cv::AKAZE::create();
  std::cout << "Orb created!" << std::endl;
  matcher = cv::BFMatcher::create();
  std::cout << "BFMatcher created!" << std::endl;
  std::cout << "refCols: " << refCols << std::endl;
  std::cout << "refRows: " << refRows << std::endl;
  // cv::Mat refGray = im_gray(refData, refCols, refRows);
  cv::Mat refGray = grayscale(refData, refCols, refRows, ColorSpace::RGBA);
  // cv::Mat refGray(refCols, refRows, CV_8UC1, refData);
  free(refData);
  std::cout << "Gray Image!" << std::endl;
  akaze->detectAndCompute(refGray, cv::noArray(), refKeyPts, refDescr);
  std::cout << "Reference image keypoints: " << refKeyPts.size() << std::endl;
  std::cout << "Reference image descriptors: " << refDescr.size() << std::endl;
  std::cout << "Orb Detect and Compute passed!" << std::endl;
  // std::cout << refDescr << std::endl;

  corners[0] = cvPoint(0, 0);
  corners[1] = cvPoint(refCols, 0);
  corners[2] = cvPoint(refCols, refRows);
  corners[3] = cvPoint(0, refRows);

  std::cout << "corners filled!" << std::endl;

  initialized = true;
  std::cout << initialized << std::endl;
  std::cout << "Ready!" << std::endl;
}

void WebARKitAkazeTracker::processFrameData(unsigned char *frameData,
                                            size_t frameCols,
                                            size_t frameRows) {
  cv::Mat colorFrame(frameRows, frameCols, CV_8UC4, frameData);
  cv::Mat grayFrame(frameRows, frameCols, CV_8UC1);
  cv::cvtColor(colorFrame, grayFrame, cv::COLOR_RGBA2GRAY);
  processFrame(grayFrame);
  grayFrame.release();
}

void WebARKitAkazeTracker::processFrame(cv::Mat frame) {
  if (this->_valid == true) {
    this->_valid = track(frame);
    // ARLOGi("valid tracking is: %s\n", _valid);
  } else {
    // ARLOGi("_valid is: %s\n", _valid ? "true" : "false" );
    this->_valid = resetTracking(frame);
  }
}

bool WebARKitAkazeTracker::resetTracking(cv::Mat frameCurr) {
  if (!initialized) {
    std::cout << "Reference image not found. AR is unintialized!" << std::endl;
    return NULL;
  }
  // std::cout << initialized << std::endl;
  std::cout << "Reset Tracking!" << std::endl;

  clear_output();

  cv::Mat frameDescr;
  std::vector<cv::KeyPoint> frameKeyPts;
  // std::cout << refDescr << std::endl;
  akaze->detectAndCompute(frameCurr, cv::noArray(), frameKeyPts, frameDescr);
  // std::cout << "detectAndCompute is ok..." << std::endl;
  std::vector<std::vector<cv::DMatch>> knnMatches;
  matcher->knnMatch(frameDescr, refDescr, knnMatches, 2);
  // std::cout << "knnmatch !" << std::endl;
  framePts.clear();
  std::vector<cv::Point2f> refPts;
  // find the best matches
  std::cout << "Good match ratio is: " << GOOD_MATCH_RATIO << std::endl;
  for (size_t i = 0; i < knnMatches.size(); ++i) {
    if (knnMatches[i][0].distance <
        GOOD_MATCH_RATIO * knnMatches[i][1].distance) {
      framePts.push_back(frameKeyPts[knnMatches[i][0].queryIdx].pt);
      refPts.push_back(refKeyPts[knnMatches[i][0].trainIdx].pt);
    }
  }
  // std::cout << "best matches !" << std::endl;

  // need at least 4 pts to define homography
  std::cout << "Frame points size: " << std::endl;
  std::cout << framePts.size() << std::endl;
  bool valid;
  // need to lowering the number of framePts to 4 (from 10). Now it track.
  if (framePts.size() >= 15) {
    H = cv::findHomography(refPts, framePts, cv::RANSAC);
    if ((valid = homographyValid(H))) {
      numMatches = framePts.size();
      fill_output(H);
      if (frameCurr.empty()) {
        std::cout << "frameCurr is empty!" << std::endl;
        return NULL;
      }
      framePrev = frameCurr.clone();
    }
  }

  return valid;
}

bool WebARKitAkazeTracker::track(cv::Mat frameCurr) {
  if (!initialized) {
    std::cout << "Reference image not found. AR is unintialized!" << std::endl;
    return NULL;
  }

  if (framePrev.empty()) {
    std::cout << "Tracking is uninitialized!" << std::endl;
    return NULL;
  }

  std::cout << "Start tracking!" << std::endl;
  clear_output();

  std::vector<float> err;
  std::vector<uchar> status;
  std::vector<cv::Point2f> newPts, goodPtsNew, goodPtsOld;
  bool valid;
  cv::calcOpticalFlowPyrLK(framePrev, frameCurr, framePts, newPts, status, err);

  // calculate average variance
  double mean, avg_variance = 0.0;
  double sum = 0.0;
  double diff;
  std::vector<double> diffs;

  for (size_t i = 0; i < framePts.size(); ++i) {
    if (status[i]) {
      goodPtsNew.push_back(newPts[i]);
      goodPtsOld.push_back(framePts[i]);
      diff = sqrt(pow(newPts[i].x - framePts[i].x, 2.0) +
                  pow(newPts[i].y - framePts[i].y, 2.0));
      sum += diff;
      diffs.push_back(diff);
    }
  }

  mean = sum / diffs.size();
  for (int i = 0; i < goodPtsNew.size(); ++i) {
    avg_variance += pow(diffs[i] - mean, 2);
  }
  avg_variance /= diffs.size();
  // if ((goodPtsCurr.size() > numMatches/2) && (1.75 > avg_variance)) {
  if ((goodPtsNew.size() > numMatches / 2) && (1.75 > avg_variance)) {
    cv::Mat transform = cv::estimateAffine2D(goodPtsOld, goodPtsNew);

    // add row of [0,0,1] to transform to make it 3x3
    cv::Mat row = cv::Mat::zeros(1, 3, CV_64F);
    row.at<double>(0, 2) = 1.0;
    transform.push_back(row);

    // update homography matrix
    H = transform * H;

    // set old points to new points
    framePts = goodPtsNew;

    if ((valid = homographyValid(H))) {
      fill_output(H);
    }
  }

  framePrev = frameCurr.clone();

  return valid;
}

std::vector<double> WebARKitAkazeTracker::getOutputData() { return output; }

bool WebARKitAkazeTracker::isValid() { return _valid; }

emscripten::val WebARKitAkazeTracker::getCorners() {
  emscripten::val corners = emscripten::val::array();
  for (auto i = 9; i < 17; i++) {
    corners.call<void>("push", output[i]);
  }
  return corners;
}

// private static methods

bool WebARKitAkazeTracker::homographyValid(cv::Mat H) {
  const double det = H.at<double>(0, 0) * H.at<double>(1, 1) -
                     H.at<double>(1, 0) * H.at<double>(0, 1);
  return 1 / N < fabs(det) && fabs(det) < N;
}

void WebARKitAkazeTracker::fill_output(cv::Mat H) {
  std::vector<cv::Point2f> warped(4);
  cv::perspectiveTransform(corners, warped, H);

  output[0] = H.at<double>(0, 0);
  output[1] = H.at<double>(0, 1);
  output[2] = H.at<double>(0, 2);
  output[3] = H.at<double>(1, 0);
  output[4] = H.at<double>(1, 1);
  output[5] = H.at<double>(1, 2);
  output[6] = H.at<double>(2, 0);
  output[7] = H.at<double>(2, 1);
  output[8] = H.at<double>(2, 2);

  output[9] = warped[0].x;
  output[10] = warped[0].y;
  output[11] = warped[1].x;
  output[12] = warped[1].y;
  output[13] = warped[2].x;
  output[14] = warped[2].y;
  output[15] = warped[3].x;
  output[16] = warped[3].y;
};

void WebARKitAkazeTracker::clear_output() { output.assign(17, 0.0); };