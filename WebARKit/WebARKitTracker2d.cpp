/*
 *  WebARKitTracker2d.cpp
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

#include <WebARKit/WebARKitTracker2d.h>

#if HAVE_2D
#include <WebARKit/WebARKitTrackable2d.h>
//#include "trackingSub.h"
#include <ARX/OCVT/PlanarTracker.h>

WebARKitTracker2d::WebARKitTracker2d() :
m_videoSourceIsStereo(false),
m_2DTrackerDataLoaded(false),
m_2DTrackerDetectedImageCount(0),
m_2DTracker(NULL),
m_running(false),
m_pageCount(0)
{
}

WebARKitTracker2d::~WebARKitTracker2d()
{
    terminate();
}

bool WebARKitTracker2d::initialize()
{
    if (!m_2DTracker) {
        m_2DTracker = std::make_shared<PlanarTracker>(PlanarTracker());
    }
    
    return true;
}

void WebARKitTracker2d::setMaxMarkersToTrack(int maximumNumberOfMarkersToTrack)
{
    m_2DTracker->SetMaximumNumberOfMarkersToTrack(maximumNumberOfMarkersToTrack);
}

int WebARKitTracker2d::getMaxMarkersToTrack() const
{
    return m_2DTracker->GetMaximumNumberOfMarkersToTrack();
}

bool WebARKitTracker2d::start(ARParamLT *paramLT, AR_PIXEL_FORMAT pixelFormat)
{
    if (!paramLT || pixelFormat == AR_PIXEL_FORMAT_INVALID) return false;
    
    ARLOGd("ARTracker2d::start(): Camera parameters: frame %dx%d, intrinsics:", paramLT->param.xsize, paramLT->param.ysize);
    for (int i = 0; i < 3; i++) {
        ARLOGd("[% 5.3f % 5.3f % 5.3f] [% 6.1f]\n", paramLT->param.mat[i][0], paramLT->param.mat[i][1], paramLT->param.mat[i][2], paramLT->param.mat[i][3]);
    }

    m_2DTracker->Initialise(paramLT->param);
    
    m_videoSourceIsStereo = false;
    m_running = true;

    ARLOGd("WebARKitTracker2d::start(): done.\n");
    return (true);
}

bool WebARKitTracker2d::start(ARParamLT *paramLT0, AR_PIXEL_FORMAT pixelFormat0, ARParamLT *paramLT1, AR_PIXEL_FORMAT pixelFormat1, const ARdouble transL2R[3][4])
{
    if (!paramLT1 || pixelFormat1 == AR_PIXEL_FORMAT_INVALID || !transL2R) return false;
    
    m_videoSourceIsStereo = true;
    m_running = true;
    
    return start(paramLT0, pixelFormat0);
}

bool WebARKitTracker2d::unloadTwoDData(void)
{
    m_2DTracker->RemoveAllMarkers();
    m_2DTrackerDataLoaded = false;
    m_pageCount = 0;
    return true;
}

bool WebARKitTracker2d::loadTwoDData(std::vector<WebARKitTrackable *>& trackables)
{
    // If data was already loaded, unload previously loaded data.
    if (m_2DTrackerDataLoaded) {
        ARLOGi("Reloading 2D data.\n");
        unloadTwoDData();
    } else {
        ARLOGi("Loading 2D data.\n");
    }
    for (std::vector<WebARKitTrackable *>::iterator it = trackables.begin(); it != trackables.end(); ++it) {
        if ((*it)->type == WebARKitTrackable::TwoD) {
            WebARKitTrackable2d *t = static_cast<WebARKitTrackable2d *>(*it);
            t->pageNo = m_pageCount;
            // N.B.: PlanarTracker::AddMarker takes a copy of the image data.
            m_2DTracker->AddMarker(t->m_refImage, t->datasetPathname, t->m_refImageX, t->m_refImageY, t->UID, t->TwoDScale());
            ARLOGi("'%s' assigned page no. %d.\n", t->datasetPathname, t->pageNo);
            m_pageCount++; // For 2D tracker, no fixed upper limit on number of trackables that can be loaded.
        }
    }
    
    m_2DTrackerDataLoaded = true;
    
    ARLOGi("Loading of 2D data complete.\n");
    return true;
}

bool WebARKitTracker2d::isRunning()
{
    return m_running;
}

bool WebARKitTracker2d::update(AR2VideoBufferT *buff, std::vector<WebARKitTrackable *>& trackables)
{
    ARLOGd("WebARKit::WebARKitTracker2d::update()\n");
    // Late loading of data now that we have image width and height.
    if (!m_2DTrackerDataLoaded) {
        if (!loadTwoDData(trackables)) {
            ARLOGe("Error loading 2D image tracker data.\n");
            return false;
        }
    }
    
    // need to check if it is correct also for Emscripten
    m_2DTracker->ProcessFrameData(buff->buffLuma);
    // Loop through all loaded 2D targets and match against tracking results.
    m_2DTrackerDetectedImageCount = 0;
    for (std::vector<WebARKitTrackable *>::iterator it = trackables.begin(); it != trackables.end(); ++it) {
        if ((*it)->type == WebARKitTrackable::TwoD) {
            WebARKitTrackable2d *trackable2D = static_cast<WebARKitTrackable2d *>(*it);
            
            if (m_2DTracker->IsTrackableVisible(trackable2D->UID)) {
                float transMat[3][4];
                if (m_2DTracker->GetTrackablePose(trackable2D->UID, transMat)) {
                    ARdouble *transL2R = (m_videoSourceIsStereo ? (ARdouble *)m_transL2R : NULL);
                    bool success = trackable2D->updateWithTwoDResults(transMat, (ARdouble (*)[4])transL2R);
                    m_2DTrackerDetectedImageCount++;
                } else {
                    trackable2D->updateWithTwoDResults(NULL, NULL);
                }
            } else {
                trackable2D->updateWithTwoDResults(NULL, NULL);
            }
        }
    }
    return true;
}

bool WebARKitTracker2d::update(AR2VideoBufferT *buff0, AR2VideoBufferT *buff1, std::vector<WebARKitTrackable *>& trackables)
{
    return update(buff0, trackables);
}

bool WebARKitTracker2d::stop()
{
    // Tracking thread is holding a reference to the camera parameters. Closing the
    // video source will dispose of the camera parameters, thus invalidating this reference.
    // So must stop tracking before closing the video source.
    if (m_2DTrackerDataLoaded) {
        unloadTwoDData();
    }
    m_videoSourceIsStereo = false;
    
    return true;
}

void WebARKitTracker2d::terminate()
{
    
}

WebARKitTrackable *WebARKitTracker2d::newTrackable(std::vector<std::string> config)
{
    // Minimum config length.
    if (config.size() < 1) {
        ARLOGe("Trackable config. must contain at least trackable type.\n");
        return nullptr;
    }
    
    // First token is trackable type.
    if (config.at(0).compare("2d") != 0) {
        return nullptr;
    }
    
    // Second token is path to 2D data.
    if (config.size() < 2) {
        ARLOGe("2D config. requires path to 2D data.\n");
        return nullptr;
    }
    
    // Optional 3rd parameter: scale.
    float scale = 0.0f;
    if (config.size() > 2) {
        scale = strtof(config.at(2).c_str(), NULL);
        if (scale == 0.0f) {
            ARLOGw("2D config. specified with invalid scale parameter ('%s'). Ignoring.\n", config.at(2).c_str());
        }
    }
    
    WebARKitTrackable2d *ret = new WebARKitTrackable2d();
    if (scale != 0.0f) ret->setTwoDScale(scale);
    bool ok = ret->load(config.at(1).c_str());
    if (!ok) {
        // Marker failed to load, or was not added
        delete ret;
        ret = nullptr;
    } else {
        // Trigger reload on next tracker update.
        unloadTwoDData();
    }
    return ret;
}

void WebARKitTracker2d::deleteTrackable(WebARKitTrackable **trackable_p)
{
    if (!trackable_p || !(*trackable_p)) return;
    if ((*trackable_p)->type != WebARKitTrackable::TwoD) return;

    unloadTwoDData();
    delete (*trackable_p);
    (*trackable_p) = NULL;
}

std::vector<WebARKitTrackable*> WebARKitTracker2d::loadImageDatabase(std::string fileName)
{
    std::vector<WebARKitTrackable*> loadedTrackables;
    if (m_2DTracker->LoadTrackableDatabase(fileName)) {
        std::vector<int> loadedIds = m_2DTracker->GetImageIds();
        for (int i = 0; i < loadedIds.size(); i++) {
            //Get loaded image infomation
            int loadedId = loadedIds[i];
            TrackedImageInfo info = m_2DTracker->GetTrackableImageInfo(loadedId);
            
            //Create trackable from loaded info
            WebARKitTrackable2d *ret = new WebARKitTrackable2d();
            ret->load2DData(info.fileName.c_str(), info.imageData, info.width, info.height);
            ret->setTwoDScale(info.scale);
            ret->pageNo = i;
            //Change ID to the Trackable generated UID
            m_2DTracker->ChangeImageId(loadedId, ret->UID);
            loadedTrackables.push_back(ret);
        }
    }
    
    if (loadedTrackables.size() > 0) {
        m_2DTrackerDataLoaded = true;
    }
    return loadedTrackables;
}

bool WebARKitTracker2d::saveImageDatabase(std::string filename)
{
    return m_2DTracker->SaveTrackableDatabase(filename);
}

void WebARKitTracker2d::setDetectorType(int detectorType)
{
    if(unloadTwoDData()) {
        m_2DTracker->SetFeatureDetector(detectorType);
    }
}

int WebARKitTracker2d::getDetectorType(void)
{
    return m_2DTracker->GetFeatureDetector();
}

#endif // HAVE_2D

