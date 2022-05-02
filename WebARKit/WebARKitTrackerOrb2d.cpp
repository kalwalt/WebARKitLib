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

#include <WebARKit/WebARKitTrackerOrb2d.h>

#if HAVE_2D
#include <WebARKit/WebARKitTrackableOrb2d.h>
#include <ARX/OCVT/PlanarOrbTracker.h>

WebARKitTrackerOrb2d::WebARKitTrackerOrb2d() :
m_videoSourceIsStereo(false),
m_Orb2DTrackerDataLoaded(false),
m_Orb2DTrackerDetectedImageCount(0),
m_Orb2DTracker(NULL),
m_running(false)
{
}

WebARKitTrackerOrb2d::~WebARKitTrackerOrb2d()
{
    terminate();
}

bool WebARKitTrackerOrb2d::initialize()
{
    if (!m_Orb2DTracker) {
        m_Orb2DTracker = std::make_shared<PlanarOrbTracker>(PlanarOrbTracker());
    }

    return true;
}

void WebARKitTrackerOrb2d::setTwoDMultiMode(bool on)
{
    m_twoDMultiMode = on;
}

bool WebARKitTrackerOrb2d::TwoDMultiMode() const
{
    return m_twoDMultiMode;
}

bool WebARKitTrackerOrb2d::start(ARParamLT *paramLT, AR_PIXEL_FORMAT pixelFormat)
{
    if (!paramLT || pixelFormat == AR_PIXEL_FORMAT_INVALID) return false;

    m_cameraXSize = paramLT->param.xsize;
    m_cameraYSize = paramLT->param.ysize;

    m_Orb2DTracker->Initialise(m_cameraXSize,m_cameraYSize, paramLT->param.mat);

    m_videoSourceIsStereo = false;
    m_running = true;

    ARLOGd("WebARKitTrackerOrb2d::start(): done.\n");
    return (true);
}

bool WebARKitTrackerOrb2d::start(ARParamLT *paramLT0, AR_PIXEL_FORMAT pixelFormat0, ARParamLT *paramLT1, AR_PIXEL_FORMAT pixelFormat1, const ARdouble transL2R[3][4])
{
    if (!paramLT1 || pixelFormat1 == AR_PIXEL_FORMAT_INVALID || !transL2R) return false;

    m_videoSourceIsStereo = true;
    m_running = true;

    return start(paramLT0, pixelFormat0);
}

bool WebARKitTrackerOrb2d::unloadTwoDData(void)
{
    m_Orb2DTracker->RemoveAllMarkers();
    m_Orb2DTrackerDataLoaded = false;
    return true;
}

bool WebARKitTrackerOrb2d::loadTwoDData(std::vector<WebARKitTrackable *>& trackables)
{
    // If data was already loaded, stop KPM tracking thread and unload previously loaded data.
    if (m_Orb2DTrackerDataLoaded) {
        ARLOGi("Reloading 2D data.\n");
        unloadTwoDData();
    } else {
        ARLOGi("Loading 2D data.\n");
    }
    int pageCount = 0;
    for (std::vector<WebARKitTrackable *>::iterator it = trackables.begin(); it != trackables.end(); ++it) {
        if ((*it)->type == WebARKitTrackable::OrbTwoD) {
            ((WebARKitTrackableOrb2d *)(*it))->pageNo = pageCount;
            // N.B.: AddMarker takes a copy of the image data.
            m_Orb2DTracker->AddMarker(((WebARKitTrackableOrb2d *)(*it))->m_refImage.get(),((WebARKitTrackableOrb2d *)(*it))->datasetPathname,((WebARKitTrackableOrb2d *)(*it))->m_refImageX,((WebARKitTrackableOrb2d *)(*it))->m_refImageY,((WebARKitTrackableOrb2d *)(*it))->UID, ((WebARKitTrackableOrb2d *)(*it))->TwoDScale());
            ARLOGi("'%s' assigned page no. %d.\n", ((WebARKitTrackableOrb2d *)(*it))->datasetPathname, pageCount);
            pageCount++;
        }
    }

    m_Orb2DTrackerDataLoaded = true;

    ARLOGi("Loading of 2D data complete.\n");
    return true;
}

bool WebARKitTrackerOrb2d::isRunning()
{
    return m_running;
}

bool WebARKitTrackerOrb2d::update(AR2VideoBufferT *buff, std::vector<WebARKitTrackable *>& trackables)
{
    ARLOGd("WebARKit::WebARKitTrackerOrb2d::update()\n");
    // Late loading of data now that we have image width and height.
    if (!m_Orb2DTrackerDataLoaded) {
        if (!loadTwoDData(trackables)) {
            ARLOGe("Error loading 2D image tracker data.\n");
            return false;
        }
    }

    m_Orb2DTracker->ProcessFrameData(buff->buff);
    // Loop through all loaded 2D targets and match against tracking results.
    m_Orb2DTrackerDetectedImageCount = 0;
    for (std::vector<WebARKitTrackable *>::iterator it = trackables.begin(); it != trackables.end(); ++it) {
        if ((*it)->type == WebARKitTrackable::OrbTwoD) {
            WebARKitTrackableOrb2d *trackableOrb2D = static_cast<WebARKitTrackableOrb2d *>(*it);
            bool trackableOrb2DFound = false;
            if (m_Orb2DTracker->IsTrackableVisible(trackableOrb2D->UID)) {
                float* transMat = m_Orb2DTracker->GetTrackablePose(trackableOrb2D->UID);
                if (transMat) {
                    ARLOGi("transMat ok\n");
                    ARdouble *transL2R = (m_videoSourceIsStereo ? (ARdouble *)m_transL2R : NULL);
                    bool success = ((WebARKitTrackableOrb2d *)(*it))->updateWithTwoDResults(trackableOrb2D->pageNo, (float (*)[4])transMat, (ARdouble (*)[4])transL2R);
                    m_Orb2DTrackerDetectedImageCount++;
                    trackableOrb2DFound = true;
                } else {
                    trackableOrb2D->updateWithTwoDResults(-1, NULL, NULL);
                }
            } else {
                trackableOrb2D->updateWithTwoDResults(-1, NULL, NULL);
            }
        }
    }
    return true;
}

bool WebARKitTrackerOrb2d::update(AR2VideoBufferT *buff0, AR2VideoBufferT *buff1, std::vector<WebARKitTrackable *>& trackables)
{
    return update(buff0, trackables);
}

bool WebARKitTrackerOrb2d::stop()
{
    // Tracking thread is holding a reference to the camera parameters. Closing the
    // video source will dispose of the camera parameters, thus invalidating this reference.
    // So must stop tracking before closing the video source.
    if (m_Orb2DTrackerDataLoaded) {
        unloadTwoDData();
    }
    m_videoSourceIsStereo = false;

    return true;
}

void WebARKitTrackerOrb2d::terminate()
{

}

WebARKitTrackable *WebARKitTrackerOrb2d::newTrackable(std::vector<std::string> config)
{
    // Minimum config length.
    if (config.size() < 1) {
        ARLOGe("Trackable config. must contain at least trackable type.\n");
        return nullptr;
    }

    // First token is trackable type.
    if (config.at(0).compare("orb_2d") != 0) {
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

    WebARKitTrackableOrb2d *ret = new WebARKitTrackableOrb2d();
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

void WebARKitTrackerOrb2d::deleteTrackable(WebARKitTrackable **trackable_p)
{
    if (!trackable_p || !(*trackable_p)) return;
    if ((*trackable_p)->type != WebARKitTrackable::OrbTwoD) return;

    m_Orb2DTracker->RemoveAllMarkers();
    delete (*trackable_p);
    (*trackable_p) = NULL;

    unloadTwoDData();
}

std::vector<WebARKitTrackable*> WebARKitTrackerOrb2d::loadImageDatabase(std::string fileName)
{
    std::vector<WebARKitTrackable*> loadedTrackables;
    /*if (m_Orb2DTracker->LoadTrackableDatabase(fileName)) {
        std::vector<int> loadedIds = m_Orb2DTracker->GetImageIds();
        for (int i = 0; i < loadedIds.size(); i++) {
            //Get loaded image infomation
            int loadedId = loadedIds[i];
            TrackedImageInfo info = m_Orb2DTracker->GetTrackableImageInfo(loadedId);

            //Create trackable from loaded info
            WebARKitTrackableOrb2d *ret = new WebARKitTrackableOrb2d();
            ret->load2DData(info.fileName.c_str(), info.imageData, info.width, info.height);
            ret->setTwoDScale(info.scale);
            ret->pageNo = i;
            //Change ID to the Trackable generated UID
            m_Orb2DTracker->ChangeImageId(loadedId, ret->UID);
            loadedTrackables.push_back(ret);
        }
    }

    if (loadedTrackables.size() > 0) {
        m_Orb2DTrackerDataLoaded = true;
    }*/
    return loadedTrackables;
}

bool WebARKitTrackerOrb2d::saveImageDatabase(std::string filename)
{
    //return m_Orb2DTracker->SaveTrackableDatabase(filename);
    return true;
}

void WebARKitTrackerOrb2d::setDetectorType(int detectorType)
{
    if(unloadTwoDData()) {
        m_Orb2DTracker->SetFeatureDetector(detectorType);
    }
}
#endif // HAVE_2D
