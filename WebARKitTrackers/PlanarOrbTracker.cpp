/*
 *  PlanarOrbTracker.cpp
 *  artoolkitX
 *
 *  A C++ class implementing the artoolkitX square fiducial marker tracker.
 *
 *  This file is part of artoolkitX.
 *
 *  artoolkitX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  artoolkitX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with artoolkitX.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2018 Realmax, Inc.
 *  Copyright 2015 Daqri, LLC.
 *  Copyright 2010-2015 ARToolworks, Inc.
 *
 *  Author(s): Philip Lamb, Daniel Bell.
 *
 */

#include "PlanarOrbTracker.h"

#include <ARX/OCVT/OCVConfig.h>
#include <ARX/OCVT/HarrisDetector.h>
#include "OrbTrackableInfo.h"
#include <ARX/OCVT/HomographyInfo.h>
//#include "OCVUtils.h" // temporary not including because duplicate symbol err. (about Points)
#include <opencv2/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

class PlanarOrbTracker::PlanarOrbTrackerImpl
{
private:
    const int MAX_FEATURES = 2000;
    const int N = 10;

    HarrisDetector _harrisDetector;
    std::vector<cv::Mat> _pyramid, _prevPyramid;
    cv::Mat refDescr;

    std::vector<OrbTrackableInfo> _trackables;

    int _currentlyTrackedMarkers;
    int _resetCount;
    int _frameCount;
    int _frameSizeX;
    int _frameSizeY;
    cv::Mat _K;
    cv::Mat _H;
    cv::Mat prevIm;
    bool _valid;
    int numMatches;
    bool initialized;
    std::vector<cv::Point2f> framePts;
    std::vector<cv::KeyPoint> refKeyPts;
    int _selectedFeatureDetectorType;
public:
    PlanarOrbTrackerImpl()
    {
        SetFeatureDetector(defaultDetectorType);
        _harrisDetector = HarrisDetector();
        _currentlyTrackedMarkers = 0;
        _frameCount = 0;
        _resetCount = 30;
        _frameSizeX = 0;
        _frameSizeY = 0;
        _K = cv::Mat();
        numMatches = 0;
        initialized = false;
        _valid = false;
    }

    void Initialise(int xFrameSize, int yFrameSize, ARdouble cParam[][4])
    {
        _frameSizeX = xFrameSize;
        _frameSizeY = yFrameSize;
        _K = cv::Mat(3, 3, CV_64FC1);
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                _K.at<double>(i,j) = (double)(cParam[i][j]);
            }
        }
    }

    bool homographyValid(cv::Mat H) {
      const double det = H.at<double>(0,0)*H.at<double>(1,1)-H.at<double>(1,0)*H.at<double>(0,1);
      return 1/N < fabs(det) && fabs(det) < N;
    }

    bool resetTracking(cv::Mat frame, size_t cols, size_t rows)
    {
        //ARLOGi("initialized is: %s\n", initialized ? "true" : "false");
        if (!IsImageInitialized()) {
           std::cout << "Reference image not found!" << std::endl;
           return NULL;
        }

    cv::Mat frameDescr;
    std::vector<cv::KeyPoint> frameKeyPts;
    std::vector<std::vector<cv::DMatch>> knnMatches;

    for(int i=0;i<_trackables.size(); i++) {
    _trackables[i]._orb->detectAndCompute(frame, cv::noArray(), frameKeyPts, frameDescr);    
    _trackables[i]._matcher->knnMatch(frameDescr, refDescr, knnMatches, 2);
    }
    //std::cout << "knnMatches:\n" << knnMatches.size() << std::endl;

    framePts.clear();
    std::vector<cv::Point2f> refPts;
    // find the best matches
    for (size_t i = 0; i < knnMatches.size(); ++i) {
        if (knnMatches[i][0].distance < nn_match_ratio * knnMatches[i][1].distance) {
            framePts.push_back( frameKeyPts[knnMatches[i][0].queryIdx].pt );
            refPts.push_back( refKeyPts[knnMatches[i][0].trainIdx].pt );
        }
    }
    bool valid;
    // need at least 4 pts to define homography
    //ARLOGi("framePts.size: %d\n", framePts.size());
    if (framePts.size() > 15) {
        _H = cv::findHomography(refPts, framePts, cv::RANSAC);
        if ( (valid = homographyValid(_H)) ) {
            numMatches = framePts.size();
            //ARLOGi("num matches: %d\n", numMatches);
            for(int i=0;i<_trackables.size(); i++) {
               _trackables[i]._trackSelection.SelectPoints();
               _trackables[i]._trackSelection.SetHomography(_H);
               _trackables[i]._isDetected = true;
            }
            prevIm = frame.clone();
        }
    }

    return valid;
    };

    bool track(cv::Mat frame, size_t frameCols, size_t frameRows) {
      if (!IsImageInitialized()) {
        std::cout << "Reference image not found!" << std::endl;
        return NULL;
      }

      if (prevIm.empty()) {
        std::cout << "Tracking is uninitialized!" << std::endl;
        return false;
      }

      // use optical flow to track keypoints
      std::vector<float> err;
      std::vector<uchar> status;
      std::vector<cv::Point2f> currPts, goodPtsCurr, goodPtsPrev;
      calcOpticalFlowPyrLK(prevIm, frame, framePts, currPts, status, err);

      // calculate average variance
      double mean, avg_variance = 0.0;
      double sum = 0.0;
      double diff;
      std::vector<double> diffs;
      for (size_t i = 0; i < framePts.size(); ++i) {
        if (status[i]) {
          goodPtsCurr.push_back(currPts[i]);
          goodPtsPrev.push_back(framePts[i]);

          diff = sqrt(pow(currPts[i].x - framePts[i].x, 2.0) +
                pow(currPts[i].y - framePts[i].y, 2.0));
          sum += diff;
          diffs.push_back(diff);
        }
      }

      mean = sum / diffs.size();
      for (int i = 0; i < goodPtsCurr.size(); ++i) {
        avg_variance += pow(diffs[i] - mean, 2);
      }
      avg_variance /= diffs.size();
      bool valid;
      if ((goodPtsCurr.size() > numMatches / 2) && (1.75 > avg_variance)) {
        cv::Mat transform = estimateAffine2D(goodPtsPrev, goodPtsCurr);

        // add row of {0,0,1} to transform to make it 3x3
        cv::Mat row = cv::Mat::zeros(1, 3, CV_64F);
        row.at<double>(0, 2) = 1.0;
        transform.push_back(row);

        // update homography matrix
        _H = transform * _H;

        // set old points to new points
        framePts = goodPtsCurr;

        if ((valid = homographyValid(_H))) {
          for(int i=0;i<_trackables.size(); i++) {
               _trackables[i]._trackSelection.SelectPoints();
               _trackables[i]._trackSelection.SetHomography(_H);
               _trackables[i]._isTracking = true;
            }
        }
      } 

      //std::cout << "preparing to copy" << std::endl;
      prevIm = frame.clone();

      for(int i=0;i<_trackables.size(); i++) {
          if((_trackables[i]._isDetected)||(_trackables[i]._isTracking)) {

              std::vector<cv::Point2f> imgPoints = _trackables[i]._trackSelection.GetSelectedFeaturesWarped();
              std::vector<cv::Point3f> objPoints = _trackables[i]._trackSelection.GetSelectedFeatures3d();
              //ARLOGi("start pose matrix\n");
              CameraPoseFromPoints(_trackables[i]._pose, objPoints, imgPoints);
          }
      }

      //ARLOGi("valid from track is: %s\n", valid ? "true" : "false" );

      return valid;
    }

    void ProcessFrameData(unsigned char * frame)
    {
        // When using emscripten the image comes in as RGB image from the browser
        // Convert it to Gray
        #if ARX_TARGET_PLATFORM_EMSCRIPTEN
          //cv::Mat colorFrame(_frameSizeY, _frameSizeX, CV_8UC4, frame);
          //cv::Mat grayFrame(_frameSizeY, _frameSizeX, CV_8UC1);
          cv::Mat grayFrame(_frameSizeY, _frameSizeX, CV_8UC1, frame);
          //cv::cvtColor(colorFrame, grayFrame, cv::COLOR_RGBA2GRAY);
          ProcessFrame(grayFrame);
          grayFrame.release();
        #else
          cv::Mat newFrame(_frameSizeY, _frameSizeX, CV_8UC1, frame);
          ProcessFrame(newFrame);
          newFrame.release();
        #endif

    }

    void ProcessFrame(cv::Mat frame)
    {
        if(!_valid) {
            _valid = resetTracking(frame, _frameSizeY, _frameSizeX);
            //ARLOGi("valid tracking is: %s\n", _valid);
        }
        //ARLOGi("_valid is: %s\n", _valid ? "true" : "false" );
        _valid =  track(frame, _frameSizeY, _frameSizeX);
    }

    void RemoveAllMarkers()
    {
        for(int i=0;i<_trackables.size(); i++) {
            _trackables[i].CleanUp();
        }
        _trackables.clear();
    }


    void AddMarker(unsigned char* buff, std::string fileName, int width, int height, int uid, float scale)
    {
        std::cout << "Add Marker" << std::endl;
        OrbTrackableInfo newTrackable;
        #if ARX_TARGET_PLATFORM_EMSCRIPTEN
          std::cout << "Add Marker EM" << std::endl;
          ARLOGi("Image width is: %d\n", width);
          ARLOGi("Image height is: %d\n", height);
          cv::Mat colorImage(height, width, CV_8UC4, buff);
          cv::Mat grayImage(height, width, CV_8UC1);
          cv::cvtColor(colorImage, grayImage, cv::COLOR_RGBA2GRAY);
          newTrackable._image = grayImage;
        #else
          newTrackable._image = cv::Mat(height, width, CV_8UC1, buff);
        #endif
        std::cout << "Add Marker _image" << std::endl;
        if(!newTrackable._image.empty()) {
            newTrackable._id = uid;
            newTrackable._fileName = fileName;
            newTrackable._scale = scale;
            newTrackable._width = newTrackable._image.cols;
            newTrackable._height = newTrackable._image.rows;
            std::cout << "Add Marker DetectFeatures" << std::endl;
            //newTrackable._featurePoints = _featureDetector.DetectAndCompute(newTrackable._image, cv::Mat(), newTrackable._descriptors);
            std::cout << "Add Marker CalcDescriptors" << std::endl;
            // newTrackable._descriptors = _featureDetector.CalcDescriptors(newTrackable._image, newTrackable._featurePoints);
            std::cout << "Add Marker FindCorners" << std::endl;
            newTrackable._cornerPoints = _harrisDetector.FindCorners(newTrackable._image);
            newTrackable._orb = cv::ORB::create(MAX_FEATURES);
            newTrackable._orb->detectAndCompute(newTrackable._image, cv::noArray(), refKeyPts, refDescr);
            newTrackable._matcher = cv::BFMatcher::create();
            std::cout << "BFMatcher created!" << std::endl;
            newTrackable._bBox.push_back(cv::Point2f(0,0));
            newTrackable._bBox.push_back(cv::Point2f(newTrackable._width, 0));
            newTrackable._bBox.push_back(cv::Point2f(newTrackable._width, newTrackable._height));
            newTrackable._bBox.push_back(cv::Point2f(0, newTrackable._height));
            newTrackable._isTracking = false;
            newTrackable._isDetected = false;
            newTrackable._resetTracks = false;
            newTrackable._trackSelection = TrackingPointSelector(newTrackable._cornerPoints, newTrackable._width, newTrackable._height, markerTemplateWidth);

            _trackables.push_back(newTrackable);
            //initialized = true;
            std::cout << "Marker Added" << std::endl;
        }
        initialized = true;
    }

    bool IsImageInitialized()
    {
        return initialized;
    }

    float* GetTrackablePose(int trackableId)
    {
        for(int i=0;i<_trackables.size(); i++) {
            if(_trackables[i]._id == trackableId) {
                cv::Mat pose = _trackables[i]._pose;
                cv::Mat poseOut;
                pose.convertTo(poseOut, CV_32FC1);
                // std::cout << "poseOut" << std::endl;
                // std::cout << poseOut << std::endl;
                return poseOut.ptr<float>(0);
            }
        }
        return NULL;
    }

    bool IsTrackableVisible(int trackableId)
    {
        for(int i=0;i<_trackables.size(); i++) {
            if(_trackables[i]._id==trackableId) {
                if((_trackables[i]._isDetected) || (_trackables[i]._isTracking)) {
                    return true;
                }
            }
        }
        return false;
    }

    void CameraPoseFromPoints(cv::Mat& pose, std::vector<cv::Point3f> oPts, std::vector<cv::Point2f> iPts)
    {
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector
        //std::cout << "Inside cameraPose" << std::endl;
        // --llvm-lto 1 compiler setting breaks the solvePnPRansac function on iOS but using the solvePnP function is faster anyways
        #if ARX_TARGET_PLATFORM_EMSCRIPTEN
          //std::cout << "before solvePnP" << std::endl;
          bool solvePnP = cv::solvePnP(oPts, iPts, _K, cv::noArray(), rvec, tvec);
          //std::cout << "solvePnP" << std::endl;
        #else
          bool solvePnP = cv::solvePnPRansac(oPts, iPts, _K, cv::noArray(), rvec, tvec);
          //std::cout << "solvePnPRansac" << std::endl;
        #endif
        if(solvePnP) {
          cv::Mat rMat;
          Rodrigues(rvec, rMat);
          cv::hconcat(rMat, tvec, pose);
          std::cout << "pose tracking is:" << std::endl;
          std::cout << pose << std::endl;
        }
    }

    bool ChangeImageId(int prevId, int newId)
    {
        for(int i=0;i<_trackables.size(); i++) {
            if(_trackables[i]._id==prevId) {
                _trackables[i]._id = newId;
                return true;
            }
        }
        return false;
    }
    std::vector<int> GetImageIds()
    {
        std::vector<int> imageIds;
        for(int i=0;i<_trackables.size(); i++) {
            imageIds.push_back(_trackables[i]._id);
        }
        return imageIds;
    }
    TrackedOrbImageInfo GetTrackableImageInfo(int trackableId)
    {
        TrackedOrbImageInfo info;
        for(int i=0;i<_trackables.size(); i++) {
            if(_trackables[i]._id==trackableId) {
                info.uid = _trackables[i]._id;
                info.scale = _trackables[i]._scale;
                info.fileName = _trackables[i]._fileName;
                // Copy the image data and use a shared_ptr to refer to it.
                unsigned char *data = (unsigned char *)malloc(_trackables[i]._width * _trackables[i]._height);
                memcpy(data, _trackables[i]._image.ptr(), _trackables[i]._width * _trackables[i]._height);
                info.imageData.reset(data, free);
                info.width = _trackables[i]._width;
                info.height = _trackables[i]._height;
                info.fileName = _trackables[i]._fileName;
                return info;
            }
        }
        return info;
    }


    void SetFeatureDetector(int detectorType)
    {
        _selectedFeatureDetectorType = detectorType;
       // _featureDetector.SetFeatureDetector(detectorType);
    }

};
PlanarOrbTracker::PlanarOrbTracker() : _trackerImpl(new PlanarOrbTrackerImpl())
{
}

PlanarOrbTracker::~PlanarOrbTracker() = default;
PlanarOrbTracker::PlanarOrbTracker(PlanarOrbTracker&&) = default;
PlanarOrbTracker& PlanarOrbTracker::operator=(PlanarOrbTracker&&) = default;

void PlanarOrbTracker::Initialise(int xFrameSize, int yFrameSize, ARdouble cParam[][4])
{
    _trackerImpl->Initialise(xFrameSize, yFrameSize, cParam);
}

void PlanarOrbTracker::ProcessFrameData(unsigned char * frame)
{
    _trackerImpl->ProcessFrameData(frame);
}

void PlanarOrbTracker::RemoveAllMarkers()
{
    _trackerImpl->RemoveAllMarkers();
}

void PlanarOrbTracker::AddMarker(unsigned char* buff, std::string fileName, int width, int height, int uid, float scale)
{
    _trackerImpl->AddMarker(buff, fileName, width, height, uid, scale);
}

bool PlanarOrbTracker::IsImageInitialized()
{
    _trackerImpl->IsImageInitialized();
}


float* PlanarOrbTracker::GetTrackablePose(int trackableId)
{
    return _trackerImpl->GetTrackablePose(trackableId);
}

bool PlanarOrbTracker::IsTrackableVisible(int trackableId)
{
    return _trackerImpl->IsTrackableVisible(trackableId);
}
/*
bool PlanarOrbTracker::LoadTrackableDatabase(std::string fileName)
{
    return _trackerImpl->LoadTrackableDatabase(fileName);
}
bool PlanarOrbTracker::SaveTrackableDatabase(std::string fileName)
{
    return _trackerImpl->SaveTrackableDatabase(fileName);
}*/

bool PlanarOrbTracker::ChangeImageId(int prevId, int newId)
{
    return _trackerImpl->ChangeImageId(prevId, newId);
}
std::vector<int> PlanarOrbTracker::GetImageIds()
{
    return _trackerImpl->GetImageIds();
}

TrackedOrbImageInfo PlanarOrbTracker::GetTrackableImageInfo(int trackableId)
{
    return _trackerImpl->GetTrackableImageInfo(trackableId);
}

void PlanarOrbTracker::SetFeatureDetector(int detectorType)
{
    _trackerImpl->SetFeatureDetector(detectorType);
}
