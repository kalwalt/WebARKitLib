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

#include "OCVConfig.h"
//#include "OCVFeatureDetector.h"
#include "OrbFeatureDetector.h"
#include "HarrisDetector.h"
#include "TrackableInfo.h"
#include "HomographyInfo.h"
//#include "OCVUtils.h" // temporary not including because duplicate symbol err. (about Points)
#include <opencv2/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#define N   10
#define GOOD_MATCH_RATIO    0.7f


class PlanarOrbTracker::PlanarOrbTrackerImpl
{
private:
    OrbFeatureDetector _featureDetector;
    HarrisDetector _harrisDetector;
    std::vector<cv::Mat> _pyramid, _prevPyramid;
    
    std::vector<TrackableInfo> _trackables;
    
    int _currentlyTrackedMarkers;
    int _resetCount;
    int _frameCount;
    int _frameSizeX;
    int _frameSizeY;
    cv::Mat _K;
    cv::Mat _H;
    cv::Mat prevIm;
    output_t *output;
    bool _valid;
    int numMatches;
    bool initialized;
    std::vector<cv::Point2f> framePts;
    std::vector<cv::KeyPoint> refKeyPts;
    std::vector<cv::Point2f> corners;   
    int _selectedFeatureDetectorType;
public:
    PlanarOrbTrackerImpl()
    {
        _featureDetector = OrbFeatureDetector();
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
        corners = std::vector<cv::Point2f>(4);
    }

    output_t *create_output() {
    output_t *output = new output_t;
    output->data = new double[OUTPUT_SIZE];
    return output;
    }
    
    void Initialise(int xFrameSize, int yFrameSize, ARdouble cParam[][4])
    {
        _frameSizeX = xFrameSize;
        _frameSizeY = yFrameSize;
        _K = cv::Mat(3,3, CV_64FC1);
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                _K.at<double>(i,j) = (double)(cParam[i][j]);
            }
        }
        /*corners[0] = cvPoint( 0, 0 );
        corners[1] = cvPoint( refCols, 0 );
        corners[2] = cvPoint( refCols, refRows );
        corners[3] = cvPoint( 0, refRows );*/
        corners[0] = cvPoint( 0, 0 );
        corners[1] = cvPoint( xFrameSize, 0 );
        corners[2] = cvPoint( xFrameSize, yFrameSize );
        corners[3] = cvPoint( 0, yFrameSize );

        output = create_output();

        ARLOGi("output is: %d\n", output->data[0]);
    }

    void clear_output()
    {
        memset(output, 0, sizeof(output_t));
    }

    void fill_output(cv::Mat H, bool valid)
    {
        std::vector<cv::Point2f> warped(4);
        cv::perspectiveTransform(corners, warped, H);

        //output->valid = valid;

        output->data[0] = H.at<double>(0,0);
        output->data[1] = H.at<double>(0,1);
        output->data[2] = H.at<double>(0,2);
        output->data[3] = H.at<double>(1,0);
        output->data[4] = H.at<double>(1,1);
        output->data[5] = H.at<double>(1,2);
        output->data[6] = H.at<double>(2,0);
        output->data[7] = H.at<double>(2,1);
        output->data[8] = H.at<double>(2,2);

        output->data[9]  = warped[0].x;
        output->data[10] = warped[0].y;
        output->data[11] = warped[1].x;
        output->data[12] = warped[1].y;
        output->data[13] = warped[2].x;
        output->data[14] = warped[2].y;
        output->data[15] = warped[3].x;
        output->data[16] = warped[3].y;
    }

    bool homographyValid(cv::Mat H) {
    const double det = H.at<double>(0,0)*H.at<double>(1,1)-H.at<double>(1,0)*H.at<double>(0,1);
    return 1/N < fabs(det) && fabs(det) < N;
    }

    //bool resetTracking(uchar imageData[], size_t cols, size_t rows)
    bool resetTracking(cv::Mat frame, size_t cols, size_t rows)
    {
        ARLOGi("initialized is: %s\n", initialized ? "true" : "false");
        if (!IsImageInitialized()) {
           std::cout << "Reference image not found!" << std::endl;
           return NULL;
        }

    clear_output();

    //cv::Mat currIm = cv::Mat(rows, cols, CV_8UC1, imageData);

    cv::Mat frameDescr;
    std::vector<cv::KeyPoint> frameKeyPts;
    cv::Mat detectionFrame;
    cv::pyrDown(frame, detectionFrame, cv::Size(frame.cols/featureDetectPyramidLevel, frame.rows/featureDetectPyramidLevel));
    cv::Mat featureMask = CreateFeatureMask(detectionFrame);
    //orb->detectAndCompute(currIm, cv::noArray(), frameKeyPts, frameDescr); // from webarkit-testing
    frameKeyPts = _featureDetector.DetectAndCompute(frame, featureMask, frameDescr);

    std::vector<std::vector<cv::DMatch>> knnMatches;
    //matcher->knnMatch(frameDescr, refDescr, knnMatches, 2);
    knnMatches = _featureDetector.MatchFeatures(frameDescr, _trackables[0]._descriptors);

    framePts.clear();
    std::vector<cv::Point2f> refPts;
    // find the best matches
    for (size_t i = 0; i < knnMatches.size(); ++i) {
        if (knnMatches[i][0].distance < GOOD_MATCH_RATIO * knnMatches[i][1].distance) {
            framePts.push_back( frameKeyPts[knnMatches[i][0].queryIdx].pt );
            refPts.push_back( refKeyPts[knnMatches[i][0].trainIdx].pt );
        }
    }
    bool valid;
    // need at least 4 pts to define homography
    if (framePts.size() > 15) {
        _H = cv::findHomography(refPts, framePts, cv::RANSAC);    
        if ( (valid = homographyValid(_H)) ) {
            numMatches = framePts.size();
            fill_output(_H, valid);
            prevIm = frame.clone();
        }
    }

    return valid;
    };
    
    cv::Mat CreateFeatureMask(cv::Mat frame)
    {
        cv::Mat featureMask;
        for(int i=0;i<_trackables.size(); i++) {
            if(_trackables[i]._isDetected) {
                if(featureMask.empty()) {
                    //Only create mask if we have something to draw in it.
                    featureMask = cv::Mat::ones(frame.size(), CV_8UC1);
                }
                std::vector<std::vector<cv::Point> > contours(1);
                for(int j=0; j<4; j++) {
                    contours[0].push_back(cv::Point(_trackables[i]._bBoxTransformed[j].x/featureDetectPyramidLevel,_trackables[i]._bBoxTransformed[j].y/featureDetectPyramidLevel));
                }
                drawContours(featureMask, contours, 0, cv::Scalar(0), -1, 8);
            }
        }
        return featureMask;
    }
    
    
    
    void ProcessFrameData(unsigned char * frame)
    {
        // When using emscripten the image comes in as RGB image from the browser
        // Convert it to Gray
        #if ARX_TARGET_PLATFORM_EMSCRIPTEN
          cv::Mat colorFrame(_frameSizeY, _frameSizeX, CV_8UC4, frame);
          cv::Mat grayFrame(_frameSizeY, _frameSizeX, CV_8UC1);
          cv::cvtColor(colorFrame, grayFrame, cv::COLOR_RGBA2GRAY);
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
            _valid = resetTracking(frame, _frameSizeX, _frameSizeY);
            ARLOGi("valid tracking is: %s\n", _valid);  
        }
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
        TrackableInfo newTrackable;
        #if ARX_TARGET_PLATFORM_EMSCRIPTEN
          std::cout << "Add Marker EM" << std::endl;
          cv::Mat colorImage(height, width, CV_8UC4, buff);
          cv::Mat grayImage(_frameSizeY, _frameSizeX, CV_8UC1);
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
            newTrackable._featurePoints = _featureDetector.DetectAndCompute(newTrackable._image, cv::Mat(), newTrackable._descriptors);
            std::cout << "Add Marker CalcDescriptors" << std::endl;
            // newTrackable._descriptors = _featureDetector.CalcDescriptors(newTrackable._image, newTrackable._featurePoints);
            std::cout << "Add Marker FindCorners" << std::endl;
            newTrackable._cornerPoints = _harrisDetector.FindCorners(newTrackable._image);
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
        _featureDetector.SetFeatureDetector(detectorType);
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

