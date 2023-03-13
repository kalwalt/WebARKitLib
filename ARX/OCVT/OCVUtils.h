#ifndef OCV_UTILS_H
#define OCV_UTILS_H
#include "OCVConfig.h"

#define N 10

std::vector<cv::Point2f> Points(std::vector<cv::KeyPoint> keypoints)
{
    std::vector<cv::Point2f> res;
    for(unsigned i = 0; i < keypoints.size(); i++) {
        res.push_back(keypoints[i].pt);
    }
    return res;
}

bool homographyValid(cv::Mat H) {
      const double det = H.at<double>(0,0)*H.at<double>(1,1)-H.at<double>(1,0)*H.at<double>(0,1);
      return 1/N < fabs(det) && fabs(det) < N;
}

//Method for calculating and validating a homography matrix from a set of corresponding points.
HomographyInfo GetHomographyInliers(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2)
{
    cv::Mat inlier_mask, homography;
    std::vector<cv::DMatch> inlier_matches;
    ARLOGi("pts1.size() = %d\n", pts1.size());
    if(pts1.size() >= 4) {
        //ARLOGi("Inside findHomography\n");
        homography = findHomography(pts1, pts2,
                                    cv::RANSAC, ransac_thresh, inlier_mask);
        //std::cout << "homography is: " <<  homography << std::endl;
    }
    
    //Failed to find a homography
    if(pts1.size() < 4 || homography.empty()) {
        return HomographyInfo();
    }
/*
    const double det = homography.at<double>(0, 0) * homography.at<double>(1, 1) - homography.at<double>(1, 0) * homography.at<double>(0, 1);
    if (det < 0)
        return HomographyInfo();
    
    const double N1 = sqrt(homography.at<double>(0, 0) * homography.at<double>(0, 0) + homography.at<double>(1, 0) * homography.at<double>(1, 0));
    if (N1 > 4 || N1 < 0.1)
        return HomographyInfo();
    
    const double N2 = sqrt(homography.at<double>(0, 1) * homography.at<double>(0, 1) + homography.at<double>(1, 1) * homography.at<double>(1, 1));
    if (N2 > 4 || N2 < 0.1)
        return HomographyInfo();
    
    const double N3 = sqrt(homography.at<double>(2, 0) * homography.at<double>(2, 0) + homography.at<double>(2, 1) * homography.at<double>(2, 1));
    if (N3 > 0.002)
        return HomographyInfo();
  */
  auto valid = homographyValid(homography);
  std::vector<uchar> status;
  if(valid) {
      ARLOGi("homography is valid\n");
    
    int linliers = 0;
    for(int i = 0; i < pts1.size(); i++) {
        if((int)inlier_mask.at<uchar>(i,0)==1) {
            ARLOGi("OK.... inliers............\n");
            status.push_back((uchar)1);
            inlier_matches.push_back(cv::DMatch(i, i, 0));
            linliers++;
        }
        else {
            status.push_back((uchar)0);
        }
    } 
    
  } else {
        ARLOGi("homography is not valid\n");
        return HomographyInfo();
    }
  //Return homography and corresponding inlier point sets
  return HomographyInfo(homography, status, inlier_matches);
}

#endif
