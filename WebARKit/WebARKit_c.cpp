/*
 *  WebARKit_c.cpp
 *  artoolkitX
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
 *  Author(s): Philip Lamb, Julian Looser.
 *
 */

// ----------------------------------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------------------------------

#include <WebARKit/WebARKit_c.h>
#include <WebARKit/WebARKitController.h>
#ifdef DEBUG
#  ifdef _WIN32
#    define MAXPATHLEN MAX_PATH
#    include <direct.h>               // _getcwd()
#    define getcwd _getcwd
#  else
#    include <unistd.h>
#    include <sys/param.h>
#  endif
#endif
#include <stdio.h>
#if !ARX_TARGET_PLATFORM_WINDOWS && !ARX_TARGET_PLATFORM_WINRT
#  include <pthread.h>
#endif

// ----------------------------------------------------------------------------------------------------

#if defined(_MSC_VER) && !defined(NAN)
#  define __nan_bytes { 0, 0, 0xc0, 0x7f }
static union { unsigned char __c[4]; float __d; } __nan_union = { __nan_bytes };
#  define NAN (__nan_union.__d)
#endif

// ----------------------------------------------------------------------------------------------------

static WebARKitController *gARTK = NULL;
static ARVideoSourceInfoListT *gARVideoSourceInfoList = NULL;

// ----------------------------------------------------------------------------------------------------

void arwRegisterLogCallback(PFN_LOGCALLBACK callback)
{
    arLogSetLogger(callback, 1); // 1 -> only callback on same thread, as required e.g. by C# interop.
}

void arwSetLogLevel(const int logLevel)
{
    if (logLevel >= 0) {
        arLogLevel = logLevel;
    }
}

int arwGetLogLevel()
{
    return arLogLevel;
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  webarkit lifecycle functions
// ----------------------------------------------------------------------------------------------------

bool arwInitialiseAR()
{
    if (!gARTK) gARTK = new WebARKitController;
	return gARTK->initialiseBase();
}

bool arwGetARToolKitVersion(char *buffer, int length)
{
	if (!buffer) return false;
    if (!gARTK) return false;

	if (const char *version = gARTK->getARToolKitVersion()) {
		strncpy(buffer, version, length - 1); buffer[length - 1] = '\0';
		return true;
	}
	return false;
}

int arwGetError()
{
    if (!gARTK) return WEBARKIT_ERROR_NONE;
    return gARTK->getError();
}

bool arwChangeToResourcesDir(const char *resourcesDirectoryPath)
{
    bool ok;
#if ARX_TARGET_PLATFORM_ANDROID
    if (resourcesDirectoryPath) ok = (arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_USE_SUPPLIED_PATH, resourcesDirectoryPath, NULL) == 0);
	else ok = (arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_BEST, NULL, NULL) == 0);
#elif ARX_TARGET_PLATFORM_WINRT
	ok = false; // No current working directory in WinRT.
#else
    if (resourcesDirectoryPath) ok = (arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_USE_SUPPLIED_PATH, resourcesDirectoryPath) == 0);
	else ok = (arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_BEST, NULL) == 0);
#endif
#ifdef DEBUG
    char buf[MAXPATHLEN];
    ARLOGd("cwd is '%s'.\n", getcwd(buf, sizeof(buf)));
#endif
    return (ok);
}

bool arwStartRunning(const char *vconf, const char *cparaName)
{
    if (!gARTK) return false;
	return gARTK->startRunning(vconf, cparaName, NULL, 0);
}

bool arwStartRunningB(const char *vconf, const char *cparaBuff, const int cparaBuffLen)
{
    if (!gARTK) return false;
	return gARTK->startRunning(vconf, NULL, cparaBuff, cparaBuffLen);
}

bool arwStartRunningStereo(const char *vconfL, const char *cparaNameL, const char *vconfR, const char *cparaNameR, const char *transL2RName)
{
    if (!gARTK) return false;
	return gARTK->startRunningStereo(vconfL, cparaNameL, NULL, 0L, vconfR, cparaNameR, NULL, 0L, transL2RName, NULL, 0L);
}

bool arwStartRunningStereoB(const char *vconfL, const char *cparaBuffL, const int cparaBuffLenL, const char *vconfR, const char *cparaBuffR, const int cparaBuffLenR, const char *transL2RBuff, const int transL2RBuffLen)
{
    if (!gARTK) return false;
	return gARTK->startRunningStereo(vconfL, NULL, cparaBuffL, cparaBuffLenL, vconfR, NULL, cparaBuffR, cparaBuffLenR, NULL, transL2RBuff, transL2RBuffLen);
}

bool arwIsRunning()
{
    if (!gARTK) return false;
	return gARTK->isRunning();
}

bool arwIsInited()
{
    if (!gARTK) return false;
	return gARTK->isInited();
}

bool arwStopRunning()
{
    if (!gARTK) return false;
	return gARTK->stopRunning();
}

bool arwShutdownAR()
{
    if (gARTK) {
        delete gARTK; // Delete the artoolkitX instance to initiate shutdown.
        gARTK = NULL;
    }

    return (true);
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Video stream management
// -------------------------------------------------------------------------------------------------


bool arwGetProjectionMatrix(const float nearPlane, const float farPlane, float p[16])
{
    if (!gARTK) return false;

#ifdef ARDOUBLE_IS_FLOAT
    return gARTK->projectionMatrix(0, nearPlane, farPlane, p);
#else
    ARdouble p0[16];
    if (!gARTK->projectionMatrix(0, nearPlane, farPlane, p0)) {
        return false;
    }
    for (int i = 0; i < 16; i++) p[i] = (float)p0[i];
    return true;
#endif
}

bool arwGetProjectionMatrixStereo(const float nearPlane, const float farPlane, float pL[16], float pR[16])
{
    if (!gARTK) return false;

#ifdef ARDOUBLE_IS_FLOAT
    return (gARTK->projectionMatrix(0, nearPlane, farPlane, pL) && gARTK->projectionMatrix(1, nearPlane, farPlane, pR));
#else
    ARdouble p0L[16];
    ARdouble p0R[16];
    if (!gARTK->projectionMatrix(0, nearPlane, farPlane, p0L) || !gARTK->projectionMatrix(1, nearPlane, farPlane, p0R)) {
        return false;
    }
    for (int i = 0; i < 16; i++) pL[i] = (float)p0L[i];
    for (int i = 0; i < 16; i++) pR[i] = (float)p0R[i];
    return true;
#endif
}


bool arwGetVideoParams(int *width, int *height, int *pixelSize, char *pixelFormatStringBuffer, int pixelFormatStringBufferLen)
{
    AR_PIXEL_FORMAT pf;

    if (!gARTK) return false;
	if (!gARTK->videoParameters(0, width, height, &pf)) return false;
    if (pixelSize) *pixelSize = arUtilGetPixelSize(pf);
    if (pixelFormatStringBuffer && pixelFormatStringBufferLen > 0) {
        strncpy(pixelFormatStringBuffer, arUtilGetPixelFormatName(pf), pixelFormatStringBufferLen);
        pixelFormatStringBuffer[pixelFormatStringBufferLen - 1] = '\0'; // guarantee nul termination.
    }
    return true;
}

bool arwGetVideoParamsStereo(int *widthL, int *heightL, int *pixelSizeL, char *pixelFormatStringBufferL, int pixelFormatStringBufferLenL, int *widthR, int *heightR, int *pixelSizeR, char *pixelFormatStringBufferR, int pixelFormatStringBufferLenR)
{
    AR_PIXEL_FORMAT pfL, pfR;

    if (!gARTK) return false;
	if (!gARTK->videoParameters(0, widthL, heightL, &pfL)) return false;
	if (!gARTK->videoParameters(1, widthR, heightR, &pfR)) return false;
    if (pixelSizeL) *pixelSizeL = arUtilGetPixelSize(pfL);
    if (pixelSizeR) *pixelSizeR = arUtilGetPixelSize(pfR);
    if (pixelFormatStringBufferL && pixelFormatStringBufferLenL > 0) {
        strncpy(pixelFormatStringBufferL, arUtilGetPixelFormatName(pfL), pixelFormatStringBufferLenL);
        pixelFormatStringBufferL[pixelFormatStringBufferLenL - 1] = '\0'; // guarantee nul termination.
    }
    if (pixelFormatStringBufferR && pixelFormatStringBufferLenR > 0) {
        strncpy(pixelFormatStringBufferR, arUtilGetPixelFormatName(pfR), pixelFormatStringBufferLenR);
        pixelFormatStringBufferR[pixelFormatStringBufferLenR - 1] = '\0'; // guarantee nul termination.
    }
    return true;
}

bool arwCapture()
{
    if (!gARTK) return false;
    return (gARTK->capture());
}

bool arwUpdateAR()
{
    if (!gARTK) return false;
    return gARTK->update();
}

bool arwUpdateTexture32(uint32_t *buffer)
{
    if (!gARTK) return false;
    return gARTK->updateTextureRGBA32(0, buffer);
}

bool arwUpdateTexture32Stereo(uint32_t *bufferL, uint32_t *bufferR)
{
    if (!gARTK) return false;
    return (gARTK->updateTextureRGBA32(0, bufferL) && gARTK->updateTextureRGBA32(1, bufferR));
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Video stream drawing.
// ----------------------------------------------------------------------------------------------------
bool arwDrawVideoInit(const int videoSourceIndex)
{
    if (!gARTK) return false;

    return (gARTK->drawVideoInit(videoSourceIndex));
}

bool arwDrawVideoSettings(int videoSourceIndex, int width, int height, bool rotate90, bool flipH, bool flipV, int hAlign, int vAlign, int scalingMode, int32_t viewport[4])
{
    if (!gARTK)return false;

    return (gARTK->drawVideoSettings(videoSourceIndex, width, height, rotate90, flipH, flipV, (WebARKitVideoView::HorizontalAlignment)hAlign, (WebARKitVideoView::VerticalAlignment)vAlign, (WebARKitVideoView::ScalingMode)scalingMode, viewport));
}

bool arwDrawVideo(const int videoSourceIndex)
{
    if (!gARTK)return false;

    return (gARTK->drawVideo(videoSourceIndex));
}

bool arwDrawVideoFinal(const int videoSourceIndex)
{
    if (!gARTK) return false;

    return (gARTK->drawVideoFinal(videoSourceIndex));
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Tracking configuration
// ----------------------------------------------------------------------------------------------------

void arwSetTrackerOptionBool(int option, bool value)
{
    if (!gARTK) return;

    if (option == ARW_TRACKER_OPTION_NFT_MULTIMODE) {
#if HAVE_NFT
        gARTK->getNFTTracker()->setNFTMultiMode(value);
#endif
    } else if (option == ARW_TRACKER_OPTION_SQUARE_DEBUG_MODE) {
        gARTK->getSquareTracker()->setDebugMode(value);
    }
}

void arwSetTrackerOptionInt(int option, int value)
{
    if (!gARTK) return;

    if (option == ARW_TRACKER_OPTION_SQUARE_THRESHOLD) {
        if (value < 0 || value > 255) return;
        gARTK->getSquareTracker()->setThreshold(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_THRESHOLD_MODE) {
        gARTK->getSquareTracker()->setThresholdMode((AR_LABELING_THRESH_MODE)value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_LABELING_MODE) {
        gARTK->getSquareTracker()->setLabelingMode(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_DETECTION_MODE) {
        gARTK->getSquareTracker()->setPatternDetectionMode(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_MATRIX_CODE_TYPE) {
        gARTK->getSquareTracker()->setMatrixCodeType((AR_MATRIX_CODE_TYPE)value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_IMAGE_PROC_MODE) {
        gARTK->getSquareTracker()->setImageProcMode(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_SIZE) {
        gARTK->getSquareTracker()->setPatternSize(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_COUNT_MAX) {
        gARTK->getSquareTracker()->setPatternCountMax(value);
    } else if (option == ARW_TRACKER_OPTION_2D_TRACKER_FEATURE_TYPE) {
#if HAVE_2D
        if (value < 0 || value > 3) return;
        gARTK->get2dTracker()->setDetectorType(value);
        gARTK->getOrb2dTracker()->setDetectorType(value);
#endif
    } else if (option == ARW_TRACKER_OPTION_2D_MAXIMUM_MARKERS_TO_TRACK) {
#if HAVE_2D
        gARTK->get2dTracker()->setMaxMarkersToTrack(value);
#endif
    }
}

void arwSetTrackerOptionFloat(int option, float value)
{
    if (!gARTK) return;

    if (option == ARW_TRACKER_OPTION_SQUARE_BORDER_SIZE) {
        if (value <= 0.0f || value >= 0.5f) return;
        gARTK->getSquareTracker()->setPattRatio(1.0f - 2.0f*value); // Convert from border size to pattern ratio.
    }
}

bool arwGetTrackerOptionBool(int option)
{
    if (!gARTK) return false;

    if (option == ARW_TRACKER_OPTION_NFT_MULTIMODE) {
#if HAVE_NFT
        return  gARTK->getNFTTracker()->NFTMultiMode();
#endif
    } else if (option == ARW_TRACKER_OPTION_SQUARE_DEBUG_MODE) {
        return gARTK->getSquareTracker()->debugMode();
    }
    return false;
}

int arwGetTrackerOptionInt(int option)
{
    if (!gARTK) return (INT_MAX);

    if (option == ARW_TRACKER_OPTION_SQUARE_THRESHOLD) {
        return gARTK->getSquareTracker()->threshold();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_THRESHOLD_MODE) {
        return gARTK->getSquareTracker()->thresholdMode();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_LABELING_MODE) {
        return (int)gARTK->getSquareTracker()->labelingMode();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_DETECTION_MODE) {
        return gARTK->getSquareTracker()->patternDetectionMode();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_MATRIX_CODE_TYPE) {
        return (int)gARTK->getSquareTracker()->matrixCodeType();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_IMAGE_PROC_MODE) {
        return gARTK->getSquareTracker()->imageProcMode();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_SIZE) {
        return gARTK->getSquareTracker()->patternSize();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_COUNT_MAX) {
        return gARTK->getSquareTracker()->patternCountMax();
     } else if (option == ARW_TRACKER_OPTION_2D_TRACKER_FEATURE_TYPE) {
#if HAVE_2D
        return gARTK->get2dTracker()->getDetectorType();
#endif
    } else if (option == ARW_TRACKER_OPTION_2D_MAXIMUM_MARKERS_TO_TRACK) {
#if HAVE_2D
        return gARTK->get2dTracker()->getMaxMarkersToTrack();
#endif       
    }
    return (INT_MAX);
}

float arwGetTrackerOptionFloat(int option)
{
    if (!gARTK) return (NAN);

    if (option == ARW_TRACKER_OPTION_SQUARE_BORDER_SIZE) {
        float value = gARTK->getSquareTracker()->pattRatio();
        if (value > 0.0f && value < 1.0f) return (1.0f - value)/2.0f; // Convert from pattern ratio to border size.
    }
    return (NAN);
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Trackable management
// ---------------------------------------------------------------------------------------------

int arwAddTrackable(const char *cfg)
{
    if (!gARTK) return -1;
	return gARTK->addTrackable(cfg);
}

bool arwGetTrackables(int *count_p, ARWTrackableStatus **statuses_p)
{
    if (!gARTK) return false;
    if (!count_p) return false;

    unsigned int trackableCount = gARTK->countTrackables();
    *count_p = (int)trackableCount;
    if (statuses_p) {
        if (trackableCount == 0) *statuses_p = NULL;
        else {
            ARWTrackableStatus *st = (ARWTrackableStatus *)calloc(trackableCount, sizeof(ARWTrackableStatus));
            for (unsigned int i = 0; i < trackableCount; i++) {
                WebARKitTrackable *t = gARTK->getTrackableAtIndex(i);
                if (!t) {
                    st[i].uid = -1;
                } else {
                    st[i].uid = t->UID;
                    st[i].visible = t->visible;
#ifdef ARDOUBLE_IS_FLOAT
                    memcpy(st[i].matrix, t->transformationMatrix, 16*sizeof(float));
                    memcpy(st[i].matrixR, t->transformationMatrixR, 16*sizeof(float));
#else
                    for (int j = 0; j < 16; j++) st[i].matrix[j] = (float)t->transformationMatrix[j];
                    for (int j = 0; j < 16; j++) st[i].matrixR[j] = (float)t->transformationMatrixR[j];
#endif
                }
            }
            *statuses_p = st;
        }
    }

    return true;
}

bool arwRemoveTrackable(int trackableUID)
{
    if (!gARTK) return false;
	return gARTK->removeTrackable(trackableUID);
}

int arwRemoveAllTrackables()
{
    if (!gARTK) return 0;
	return gARTK->removeAllTrackables();
}

#if HAVE_2D
bool arwLoad2dTrackableDatabase(const char *databaseFileName)
{
    if (!gARTK) return false;
    return gARTK->load2DTrackerImageDatabase(databaseFileName);
}

bool arwSave2dTrackableDatabase(const char *databaseFileName)
{
    if (!gARTK) return false;
    return gARTK->save2DTrackerImageDatabase(databaseFileName);
}
#endif // HAVE_2D

bool arwQueryTrackableVisibilityAndTransformation(int trackableUID, float matrix[16])
{
    WebARKitTrackable *trackable;

    if (!gARTK) return false;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwQueryTrackableVisibilityAndTransformation(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return false;
    }
    for (int i = 0; i < 16; i++) matrix[i] = (float)trackable->transformationMatrix[i];
    return trackable->visible;
}

bool arwQueryTrackableVisibilityAndTransformationStereo(int trackableUID, float matrixL[16], float matrixR[16])
{
    WebARKitTrackable *trackable;

    if (!gARTK) return false;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwQueryTrackableVisibilityAndTransformationStereo(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return false;
    }
    for (int i = 0; i < 16; i++) matrixL[i] = (float)trackable->transformationMatrix[i];
    for (int i = 0; i < 16; i++) matrixR[i] = (float)trackable->transformationMatrixR[i];
    return trackable->visible;
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Trackable patterns
// ---------------------------------------------------------------------------------------------

int arwGetTrackablePatternCount(int trackableUID)
{
    WebARKitTrackable *trackable;

    if (!gARTK) return 0;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackablePatternCount(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return 0;
    }
    return (int)trackable->getPatternCount();
}

bool arwGetTrackablePatternConfig(int trackableUID, int patternID, float matrix[16], float *width, float *height, int *imageSizeX, int *imageSizeY)
{
    WebARKitTrackable *trackable;
    //WebARKitPattern *p;

    if (!gARTK) return false;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackablePatternConfig(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return false;
    }

    /*if (!(p = trackable->getPattern(patternID))) {
        ARLOGe("arwGetTrackablePatternConfig(): Trackable with UID %d has no pattern with ID %d.\n", trackableUID, patternID);
        return false;
    }*/

    if (matrix) {
        //for (int i = 0; i < 16; i++) matrix[i] = (float)p->m_matrix[i];
        ARdouble pattMtx[16];
        if (!trackable->getPatternTransform(patternID, pattMtx)) return false;
        for (int i = 0; i < 16; i++) matrix[i] = (float)pattMtx[i];
    }
    /*if (width) *width = (float)p->m_width;
    if (height) *height = (float)p->m_height;
    if (imageSizeX) *imageSizeX = p->m_imageSizeX;
    if (imageSizeY) *imageSizeY = p->m_imageSizeY;*/
    std::pair<float, float> size = trackable->getPatternSize(patternID);
    if (width) *width = size.first;
    if (height) *height = size.second;
    std::pair<int, int> imageSize = trackable->getPatternImageSize(patternID, trackable->type == WebARKitTrackable::SINGLE || trackable->type == WebARKitTrackable::MULTI || trackable->type == WebARKitTrackable::MULTI_AUTO ? gARTK->getSquareTracker()->matrixCodeType() : (AR_MATRIX_CODE_TYPE)0);
    if (imageSizeX) *imageSizeX = imageSize.first;
    if (imageSizeY) *imageSizeY = imageSize.second;
    return true;
}

bool arwGetTrackablePatternImage(int trackableUID, int patternID, uint32_t *buffer)
{
    WebARKitTrackable *trackable;
    //WebARKitPattern *p;

    if (!gARTK) return false;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackablePatternImage(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return false;
    }

    /*if (!(p = trackable->getPattern(patternID))) {
        ARLOGe("arwGetTrackablePatternImage(): Trackable with UID %d has no pattern with ID %d.\n", trackableUID, patternID);
        return false;
    }

    if (!p->m_image) {
        return false;
    }

    memcpy(buffer, p->m_image, sizeof(uint32_t) * p->m_imageSizeX * p->m_imageSizeY);
    return true;*/
    return trackable->getPatternImage(patternID, buffer, trackable->type == WebARKitTrackable::SINGLE || trackable->type == WebARKitTrackable::MULTI || trackable->type == WebARKitTrackable::MULTI_AUTO ? gARTK->getSquareTracker()->matrixCodeType() : (AR_MATRIX_CODE_TYPE)0);

}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Trackable options
// ---------------------------------------------------------------------------------------------

bool arwGetTrackableOptionBool(int trackableUID, int option)
{
    WebARKitTrackable *trackable;

    if (!gARTK) return false;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackableOptionBool(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return false;
    }

    switch (option) {
        case ARW_TRACKABLE_OPTION_FILTERED:
            return(trackable->isFiltered());
            break;
        case ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION:
            if (trackable->type == WebARKitTrackable::SINGLE) return (((WebARKitTrackableSquare *)trackable)->useContPoseEstimation);
            break;
        default:
            ARLOGe("arwGetTrackableOptionBool(): Unrecognised option %d.\n", option);
            break;
    }
    return(false);
}

void arwSetTrackableOptionBool(int trackableUID, int option, bool value)
{
    WebARKitTrackable *trackable;

    if (!gARTK) return;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwSetTrackableOptionBool(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return;
    }

    switch (option) {
        case ARW_TRACKABLE_OPTION_FILTERED:
            trackable->setFiltered(value);
            break;
        case ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION:
            if (trackable->type == WebARKitTrackable::SINGLE) ((WebARKitTrackableSquare *)trackable)->useContPoseEstimation = value;
            break;
        default:
            ARLOGe("arwSetTrackableOptionBool(): Unrecognised option %d.\n", option);
            break;
    }
}

int arwGetTrackableOptionInt(int trackableUID, int option)
{
    WebARKitTrackable *trackable;

    if (!gARTK) return INT_MIN;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackableOptionBool(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return (INT_MIN);
    }

    switch (option) {
        case ARW_TRACKABLE_OPTION_MULTI_MIN_SUBMARKERS:
            if (trackable->type == WebARKitTrackable::MULTI) return ((WebARKitTrackableMultiSquare *)trackable)->config->min_submarker;
            break;
        default:
            ARLOGe("arwGetTrackableOptionInt(): Unrecognised option %d.\n", option);
            break;
    }
    return (INT_MIN);
}

void arwSetTrackableOptionInt(int trackableUID, int option, int value)
{
    WebARKitTrackable *trackable;

    if (!gARTK) return;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwSetTrackableOptionInt(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return;
    }

    switch (option) {
        case ARW_TRACKABLE_OPTION_MULTI_MIN_SUBMARKERS:
            if (trackable->type == WebARKitTrackable::MULTI) ((WebARKitTrackableMultiSquare *)trackable)->config->min_submarker = value;
            break;
        default:
            ARLOGe("arwSetTrackableOptionInt(): Unrecognised option %d.\n", option);
            break;
    }
}

float arwGetTrackableOptionFloat(int trackableUID, int option)
{
    WebARKitTrackable *trackable;

    if (!gARTK) return (NAN);
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackableOptionBool(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return (NAN);
    }

    switch (option) {
        case ARW_TRACKABLE_OPTION_FILTER_SAMPLE_RATE:
            return ((float)trackable->filterSampleRate());
            break;
        case ARW_TRACKABLE_OPTION_FILTER_CUTOFF_FREQ:
            return ((float)trackable->filterCutoffFrequency());
            break;
        case ARW_TRACKABLE_OPTION_SQUARE_CONFIDENCE:
            if (trackable->type == WebARKitTrackable::SINGLE) return ((float)((WebARKitTrackableSquare *)trackable)->getConfidence());
            else return (NAN);
            break;
        case ARW_TRACKABLE_OPTION_SQUARE_CONFIDENCE_CUTOFF:
            if (trackable->type == WebARKitTrackable::SINGLE) return ((float)((WebARKitTrackableSquare *)trackable)->getConfidenceCutoff());
            else return (NAN);
            break;
        case ARW_TRACKABLE_OPTION_NFT_SCALE:
#if HAVE_NFT
            if (trackable->type == WebARKitTrackable::NFT) return ((float)((WebARKitTrackableNFT *)trackable)->NFTScale());
            else return (NAN);
#else
            return (NAN);
#endif
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_MATRIX:
            if (trackable->type == WebARKitTrackable::MULTI) return (float)((WebARKitTrackableMultiSquare *)trackable)->config->cfMatrixCutoff;
            else return (NAN);
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_PATTERN:
            if (trackable->type == WebARKitTrackable::MULTI) return (float)((WebARKitTrackableMultiSquare *)trackable)->config->cfPattCutoff;
            else return (NAN);
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_INLIER_PROB:
            if (trackable->type == WebARKitTrackable::MULTI) return (float)((WebARKitTrackableMultiSquare *)trackable)->config->minInlierProb;
            else return (NAN);
            break;
        default:
            ARLOGe("arwGetTrackableOptionFloat(): Unrecognised option %d.\n", option);
            break;
    }
    return (NAN);
}

void arwSetTrackableOptionFloat(int trackableUID, int option, float value)
{
    WebARKitTrackable *trackable;

    if (!gARTK) return;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwSetTrackableOptionFloat(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return;
    }

    switch (option) {
        case ARW_TRACKABLE_OPTION_FILTER_SAMPLE_RATE:
            trackable->setFilterSampleRate(value);
            break;
        case ARW_TRACKABLE_OPTION_FILTER_CUTOFF_FREQ:
            trackable->setFilterCutoffFrequency(value);
            break;
        case ARW_TRACKABLE_OPTION_SQUARE_CONFIDENCE_CUTOFF:
            if (trackable->type == WebARKitTrackable::SINGLE) ((WebARKitTrackableSquare *)trackable)->setConfidenceCutoff(value);
            break;
        case ARW_TRACKABLE_OPTION_NFT_SCALE:
#if HAVE_NFT
            if (trackable->type == WebARKitTrackable::NFT) ((WebARKitTrackableNFT *)trackable)->setNFTScale(value);
#endif
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_MATRIX:
            if (trackable->type == WebARKitTrackable::MULTI) ((WebARKitTrackableMultiSquare *)trackable)->config->cfMatrixCutoff = value;
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_PATTERN:
            if (trackable->type == WebARKitTrackable::MULTI) ((WebARKitTrackableMultiSquare *)trackable)->config->cfPattCutoff = value;
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_INLIER_PROB:
            if (trackable->type == WebARKitTrackable::MULTI) ((WebARKitTrackableMultiSquare *)trackable)->config->minInlierProb = value;
            break;
        default:
            ARLOGe("arwSetTrackableOptionFloat(): Unrecognised option %d.\n", option);
            break;
    }
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Utility
// ----------------------------------------------------------------------------------------------------
bool arwLoadOpticalParams(const char *optical_param_name, const char *optical_param_buff, const int optical_param_buffLen, const float projectionNearPlane, const float projectionFarPlane, float *fovy_p, float *aspect_p, float m[16], float p[16])
{
    if (!gARTK) return false;

#ifdef ARDOUBLE_IS_FLOAT
    return gARTK->loadOpticalParams(optical_param_name, optical_param_buff, optical_param_buffLen, projectionNearPlane, projectionFarPlane, fovy_p, aspect_p, m, p);
#else
    ARdouble fovy, aspect, m0[16], p0[16];
	if (!gARTK->loadOpticalParams(optical_param_name, optical_param_buff, optical_param_buffLen, projectionNearPlane, projectionFarPlane, &fovy, &aspect, m0, (p ? p0 : NULL))) {
        return false;
    }
    *fovy_p = (float)fovy;
    *aspect_p = (float)aspect;
    for (int i = 0; i < 16; i++) m[i] = (float)m0[i];
    if (p) for (int i = 0; i < 16; i++) p[i] = (float)p0[i];
    return true;
#endif
}

#pragma mark Web API

#if ARX_TARGET_PLATFORM_EMSCRIPTEN

int arwVideoPushInitWeb(int videoSourceIndex, int width, int height, const char *pixelFormat, int camera_index, int camera_face) {
    if (!gARTK) {
        return -1;
    }

    return gARTK->webVideoPushInitC(videoSourceIndex, width, height, pixelFormat, camera_index, camera_face);
}

#endif

// ----------------------------------------------------------------------------------------------------
#pragma mark  Video source info list management
// ----------------------------------------------------------------------------------------------------

int arwCreateVideoSourceInfoList(char *config)
{
    if (gARVideoSourceInfoList) {
        ar2VideoDeleteSourceInfoList(&gARVideoSourceInfoList);
    }
    gARVideoSourceInfoList = ar2VideoCreateSourceInfoList(config);
    if (!gARVideoSourceInfoList) {
        return 0;
    } else if (gARVideoSourceInfoList->count == 0) {
        ar2VideoDeleteSourceInfoList(&gARVideoSourceInfoList);
        return 0;
    } else {
        return gARVideoSourceInfoList->count;
    }
}

bool arwGetVideoSourceInfoListEntry(int index, char *nameBuf, int nameBufLen, char *modelBuf, int modelBufLen, char *UIDBuf, int UIDBufLen, uint32_t *flags_p, char *openTokenBuf, int openTokenBufLen)
{
    if (!gARVideoSourceInfoList) {
        return false;
    }
    if (index < 0 || index >= gARVideoSourceInfoList->count) {
        return false;
    }
    if (nameBuf && nameBufLen > 0) {
        if (!gARVideoSourceInfoList->info[index].name) *nameBuf = '\0';
        else {
            strncpy(nameBuf, gARVideoSourceInfoList->info[index].name, nameBufLen);
            nameBuf[nameBufLen - 1] = '\0';
        }
    }
    if (modelBuf && modelBufLen > 0) {
        if (!gARVideoSourceInfoList->info[index].model) *modelBuf = '\0';
        else {
            strncpy(modelBuf, gARVideoSourceInfoList->info[index].model, modelBufLen);
            modelBuf[modelBufLen - 1] = '\0';
        }
    }
    if (UIDBuf && UIDBufLen > 0) {
        if (!gARVideoSourceInfoList->info[index].UID) *UIDBuf = '\0';
        else {
            strncpy(UIDBuf, gARVideoSourceInfoList->info[index].UID, UIDBufLen);
            UIDBuf[UIDBufLen - 1] = '\0';
        }
    }
    if (flags_p) {
        *flags_p = gARVideoSourceInfoList->info[index].flags;
    }
    if (openTokenBuf && openTokenBufLen > 0) {
        if (!gARVideoSourceInfoList->info[index].open_token) *openTokenBuf = '\0';
        else {
            strncpy(openTokenBuf, gARVideoSourceInfoList->info[index].open_token, openTokenBufLen);
            openTokenBuf[openTokenBufLen - 1] = '\0';
        }
    }
    return true;
}

void arwDeleteVideoSourceInfoList(void)
{
    if (gARVideoSourceInfoList) {
        ar2VideoDeleteSourceInfoList(&gARVideoSourceInfoList);
        gARVideoSourceInfoList = NULL;
    }
}


