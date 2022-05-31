#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitOrbTracker.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h>


WebARKitOrbTracker::WebARKitOrbTracker():corners(4)
{
    output = new double[17];
}

void WebARKitOrbTracker::initialize(unsigned char * refData, size_t refCols, size_t refRows) {
    std::cout << "Start!" << std::endl;
    std::cout << MAX_FEATURES << std::endl;
    orb = cv::ORB::create(MAX_FEATURES);
    std::cout << "Orb created!" << std::endl;
    matcher = cv::BFMatcher::create();
    std::cout << "BFMatcher created!" << std::endl;
    //cv::Mat refGray = im_gray(refData, refCols, refRows);
    cv::Mat colorFrame(refCols, refRows, CV_8UC4, refData);
    free(refData);
    cv::Mat refGray(refCols, refRows, CV_8UC1);
    cv::cvtColor(colorFrame, refGray, cv::COLOR_RGBA2GRAY);
    std::cout << "Gray Image!" << std::endl;
    //std::cout << refGray << std::endl;
    orb->detectAndCompute(refGray, cv::noArray(), refKeyPts, refDescr);
    std::cout << "Orb Detect and Compute passed!" << std::endl;
    //std::cout << refDescr << std::endl;

    corners[0] = cvPoint( 0, 0 );
    corners[1] = cvPoint( refCols, 0 );
    corners[2] = cvPoint( refCols, refRows );
    corners[3] = cvPoint( 0, refRows );

    std::cout << "corners filled!" << std::endl;

    initialized = true;
    std::cout << initialized << std::endl;
    std::cout << "Ready!" << std::endl;
}

void WebARKitOrbTracker::processFrameData(unsigned char * frameData, size_t frameCols, size_t frameRows) {
    cv::Mat colorFrame(frameRows, frameCols, CV_8UC4, frameData);
    cv::Mat grayFrame(frameRows, frameCols, CV_8UC1);
    cv::cvtColor(colorFrame, grayFrame, cv::COLOR_RGBA2GRAY);
    processFrame(grayFrame);
    grayFrame.release();
}

void WebARKitOrbTracker::processFrame(cv::Mat frame)
    {
        if(!_valid) {
            _valid = resetTracking(frame);
            //ARLOGi("valid tracking is: %s\n", _valid);
        }
        //ARLOGi("_valid is: %s\n", _valid ? "true" : "false" );
        _valid =  track(frame);
    }

bool WebARKitOrbTracker::resetTracking(cv::Mat frameCurr) {
    if (!initialized) {
        std::cout << "Reference image not found. AR is unintialized!" << std::endl;
        return NULL;
    }
    //std::cout << initialized << std::endl;

    clear_output();

    cv::Mat frameDescr;
    std::vector<cv::KeyPoint> frameKeyPts;
    //std::cout << refDescr << std::endl;
    orb->detectAndCompute(frameCurr, cv::noArray(), frameKeyPts, frameDescr);
    //std::cout << "detectAndCompute is ok..." << std::endl;
    std::vector<std::vector<cv::DMatch>> knnMatches;
    matcher->knnMatch(frameDescr, refDescr, knnMatches, 2);
    //std::cout << "knnmatch !" << std::endl;
    framePts.clear();
    std::vector<cv::Point2f> refPts;
    // find the best matches
    for (size_t i = 0; i < knnMatches.size(); ++i) {
        if (knnMatches[i][0].distance < GOOD_MATCH_RATIO * knnMatches[i][1].distance) {
            framePts.push_back( frameKeyPts[knnMatches[i][0].queryIdx].pt );
            refPts.push_back( refKeyPts[knnMatches[i][0].trainIdx].pt );
        }
    }
    //std::cout << "best matches !" << std::endl;

    // need at least 4 pts to define homography
    std::cout << "Frame points size: " << std::endl;
    std::cout << framePts.size() << std::endl;
    bool valid;
    if (framePts.size() > 10) {
        H = cv::findHomography(refPts, framePts, cv::RANSAC);
        if (valid = homographyValid(H)) {
            numMatches = framePts.size();
            fill_output(H, output);
            if (frameCurr.empty()) {
                std::cout << "frameCurr is empty!" << std::endl;
                return NULL;
            }
            framePrev = frameCurr.clone();
        }
    }

    return valid;
}

bool WebARKitOrbTracker::track(cv::Mat frameCurr) {
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
    for (size_t i = 0; i < framePts.size(); ++i) {
        if (status[i]) {
            goodPtsNew.push_back(newPts[i]);
            goodPtsOld.push_back(framePts[i]);
        }
    }

    if (!goodPtsNew.empty() && goodPtsNew.size() > 2*numMatches/3) {
        cv::Mat transform = cv::estimateAffine2D(goodPtsOld, goodPtsNew);

        // add row of [0,0,1] to transform to make it 3x3
        cv::Mat row = cv::Mat::zeros(1, 3, CV_64F);
        row.at<double>(0,2) = 1.0;
        transform.push_back(row);

        // update homography matrix
        H = transform * H;

        // set old points to new points
        framePts = goodPtsNew;

        if (valid = homographyValid(H)) {
            fill_output(H, output);
        }
    }

    framePrev = frameCurr.clone();

    return valid;
}
// private static methods

bool WebARKitOrbTracker::homographyValid(cv::Mat H) {
    const double det = H.at<double>(0,0)*H.at<double>(1,1)-H.at<double>(1,0)*H.at<double>(0,1);
    return 1/N < fabs(det) && fabs(det) < N;
}

void WebARKitOrbTracker::fill_output(cv::Mat H, double *output) {
    std::vector<cv::Point2f> warped(4);
    cv::perspectiveTransform(corners, warped, H);

    output[0] = H.at<double>(0,0);
    output[1] = H.at<double>(0,1);
    output[2] = H.at<double>(0,2);
    output[3] = H.at<double>(1,0);
    output[4] = H.at<double>(1,1);
    output[5] = H.at<double>(1,2);
    output[6] = H.at<double>(2,0);
    output[7] = H.at<double>(2,1);
    output[8] = H.at<double>(2,2);

    output[9]  = warped[0].x;
    output[10] = warped[0].y;
    output[11] = warped[1].x;
    output[12] = warped[1].y;
    output[13] = warped[2].x;
    output[14] = warped[2].y;
    output[15] = warped[3].x;
    output[16] = warped[3].y;
};

void WebARKitOrbTracker::clear_output() {
    for (int i = 0; i < 17; i++) output[i] = 0;
};