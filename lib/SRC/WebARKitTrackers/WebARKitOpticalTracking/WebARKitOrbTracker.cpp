#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitOrbTracker.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>

WebARKitOrbTracker::WebARKitOrbTracker()
{
}

void WebARKitOrbTracker::initialize(uchar refData[], size_t refCols, size_t refRows) {
    std::cout << "Start!" << std::endl;
    std::cout << MAX_FEATURES << std::endl;
    orb = cv::ORB::create(MAX_FEATURES);
    std::cout << "Orb created!" << std::endl;
    matcher = cv::BFMatcher::create();

    cv::Mat refGray = im_gray(refData, refCols, refRows);
    std::cout << "Gray Image!" << std::endl;
    orb->detectAndCompute(refGray, cv::noArray(), refKeyPts, refDescr);
    std::cout << "Orb Detect and Compute passed!" << std::endl;

    corners[0] = cvPoint( 0, 0 );
    corners[1] = cvPoint( refCols, 0 );
    corners[2] = cvPoint( refCols, refRows );
    corners[3] = cvPoint( 0, refRows );

    std::cout << "corners filled!" << std::endl;

    initialized = true;
    std::cout << "Ready!" << std::endl;
}

double* WebARKitOrbTracker::resetTracking(uchar frameData[], size_t frameCols, size_t frameRows) {
    if (!initialized) {
        std::cout << "Reference image not found. AR is unintialized!" << std::endl;
        return NULL;
    }

    clear_output();

    cv::Mat frameCurr = im_gray(frameData, frameCols, frameRows);

    cv::Mat frameDescr;
    std::vector<cv::KeyPoint> frameKeyPts;
    orb->detectAndCompute(frameCurr, cv::noArray(), frameKeyPts, frameDescr);

    std::vector<std::vector<cv::DMatch>> knnMatches;
    matcher->knnMatch(frameDescr, refDescr, knnMatches, 2);

    framePts.clear();
    std::vector<cv::Point2f> refPts;
    // find the best matches
    for (size_t i = 0; i < knnMatches.size(); ++i) {
        if (knnMatches[i][0].distance < GOOD_MATCH_RATIO * knnMatches[i][1].distance) {
            framePts.push_back( frameKeyPts[knnMatches[i][0].queryIdx].pt );
            refPts.push_back( refKeyPts[knnMatches[i][0].trainIdx].pt );
        }
    }

    // need at least 4 pts to define homography
    if (framePts.size() > 10) {
        H = cv::findHomography(refPts, framePts, cv::RANSAC);
        if (homographyValid(H)) {
            numMatches = framePts.size();
            fill_output(H);
        }
    }

    framePrev = frameCurr.clone();

    return output;
}

double* WebARKitOrbTracker::track(uchar frameData[], size_t frameCols, size_t frameRows) {
    if (!initialized) {
        std::cout << "Reference image not found. AR is unintialized!" << std::endl;
        return NULL;
    }

    if (framePrev.empty()) {
        std::cout << "Tracking is uninitialized!" << std::endl;
        return NULL;
    }

    clear_output();

    cv::Mat frameCurr = im_gray(frameData, frameCols, frameRows);
    // GaussianBlur(frameCurr, frameCurr, Size(5,5), 2);

    std::vector<float> err;
    std::vector<uchar> status;
    std::vector<cv::Point2f> newPts, goodPtsNew, goodPtsOld;
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

        if (homographyValid(H)) {
            fill_output(H);
        }
    }

    framePrev = frameCurr.clone();

    return output;
}
// private static methods

cv::Mat WebARKitOrbTracker::im_gray(uchar data[], size_t cols, size_t rows) {
    uint32_t idx;
    uchar gray[rows][cols];
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            idx = (i * cols * 4) + j * 4;

            // rgba to rgb
            uchar r = data[idx];
            uchar g = data[idx + 1];
            uchar b = data[idx + 2];
            // uchar a = data[idx + 3];

            // turn frame image to gray scale
            gray[i][j] = (0.30 * r) + (0.59 * g) + (0.11 * b);
        }
    }

    return cv::Mat(rows, cols, CV_8UC1, gray);
}

bool WebARKitOrbTracker::homographyValid(cv::Mat H) {
    const double det = H.at<double>(0,0)*H.at<double>(1,1)-H.at<double>(1,0)*H.at<double>(0,1);
    return 1/N < fabs(det) && fabs(det) < N;
}

void WebARKitOrbTracker::fill_output(cv::Mat H) {
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
