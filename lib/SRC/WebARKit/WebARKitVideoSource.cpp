/*
 *  ARVideoSource.cpp
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
 *  Copyright 2015-2016 Daqri, LLC.
 *  Copyright 2010-2015 ARToolworks, Inc.
 *
 *  Author(s): Julian Looser, Philip Lamb.
 *
 */
 
#include <WebARKit/Platform.h>
#include <WebARKit/WebARKitVideoSource.h>
#include <WebARKit/Error.h>
#include <WebARKit/WebARKitController.h>
#include <ARX/ARVideo/videoRGBA.h>
#if HAVE_ARM_NEON || HAVE_ARM64_NEON
#  include <arm_neon.h>
#  ifdef ANDROID
#    include "cpu-features.h"
#  endif
#endif
#include <stdlib.h>
#include <inttypes.h>
#ifdef _WIN32
#  define _USE_MATH_DEFINES
#endif
#include <math.h>


#define MAX(x,y) (x > y ? x : y)
#define MIN(x,y) (x < y ? x : y)
#define CLAMP(x,r1,r2) (MIN(MAX(x,r1),r2))

WebARKitVideoSource::WebARKitVideoSource() :
    deviceState(DEVICE_CLOSED),
    m_vid(NULL),
    m_noCpara(false),
    cameraParam(NULL),
    cameraParamBuffer(NULL),
    cameraParamBufferLen(0L),
    cparamLT(NULL),
    videoConfiguration(NULL),
    videoWidth(0),
    videoHeight(0),
    pixelFormat((AR_PIXEL_FORMAT)(-1)),
    m_captureFrameWaitCount(0),
    m_frameBuffer(NULL),
    m_getFrameTextureTime{0, 0},
    m_error(WEBARKIT_ERROR_NONE)
{
    pthread_rwlock_init(&m_frameBufferLock, NULL);
}

WebARKitVideoSource::~WebARKitVideoSource()
{
    if (deviceState != DEVICE_CLOSED) {
        close();
    }

    if (videoConfiguration) {
        free(videoConfiguration);
        videoConfiguration = NULL;
    }
    if (cameraParam) {
        free(cameraParam);
        cameraParam = NULL;
    }
    if (cameraParamBuffer) {
        free(cameraParamBuffer);
        cameraParamBuffer = NULL;
        cameraParamBufferLen = 0;
    }

    pthread_rwlock_destroy(&m_frameBufferLock);
}

void WebARKitVideoSource::configure(const char* vconf, bool noCpara, const char* cparaName, const char* cparaBuff, size_t cparaBuffLen)
{
    if (deviceState != DEVICE_CLOSED) {
        ARLOGe("WebARKitVideoSource::open(): error: device is already open.\n");
        return;
    }

    m_noCpara = noCpara;

    free(videoConfiguration);
    videoConfiguration = NULL;
    if (vconf) {
        videoConfiguration = strdup(vconf);
        ARLOGi("Setting video configuration '%s'.\n", videoConfiguration);
    }

    free(cameraParam);
    cameraParam = NULL;
    if (cparaName) {
        cameraParam = strdup(cparaName);
        ARLOGi("Setting camera parameters file '%s'.\n", cameraParam);
    }

    free(cameraParamBuffer);
    cameraParamBuffer = NULL;
    cameraParamBufferLen = 0;
    if (cparaBuff) {
        if (cparaBuffLen) {
            cameraParamBufferLen = cparaBuffLen;
            cameraParamBuffer = (char *)malloc(sizeof(char) * cameraParamBufferLen);
            memcpy(cameraParamBuffer, cparaBuff, cameraParamBufferLen);
        }
        ARLOGi("Setting camera parameters buffer: %ld bytes.\n", cameraParamBufferLen);
    }
}

bool WebARKitVideoSource::open()
{
    ARLOGi("Opening webarkit video using configuration '%s'.\n", videoConfiguration);

    if (deviceState != DEVICE_CLOSED) {
        ARLOGe("WebARKitVideoSource::open(): error: device is already open.\n");
        return false;
    }

    // Open the video path
    if (!(m_vid = ar2VideoOpenAsync(videoConfiguration, openCallback, (void *)this))) {
        if (!(m_vid = ar2VideoOpen(videoConfiguration))) {
            ARLOGe("Error: unable to open connection to camera using configuration '%s'.\n", videoConfiguration);
            return false;
        }
        deviceState = DEVICE_OPEN;
        return this->open2();
    }
    deviceState = DEVICE_OPEN;
    return true;
}

// static callback method.
void WebARKitVideoSource::openCallback(void *userdata)
{
    if (!userdata) {
        ARLOGe("WebARKitVideoSource::openCallback: userdata was NULL.\n");
        return;
    }
    WebARKitVideoSource *vs = reinterpret_cast<WebARKitVideoSource *>(userdata);
    vs->open2();
}

bool WebARKitVideoSource::open2()
{
    // Find the size of the video
    if (ar2VideoGetSize(m_vid, &videoWidth, &videoHeight) < 0) {
        ARLOGe("Error: unable to get video size.\n");
        this->close();
        return false;
    }

    // Get the format in which the camera is returning pixels
    pixelFormat = ar2VideoGetPixelFormat(m_vid);
    if (pixelFormat < 0 ) {
        ARLOGe("Error: unable to get pixel format.\n");
        this->close();
        return false;
    }

    ARLOGi("Opened webarkit video %dx%d@%dBpp (%s).\n", videoWidth, videoHeight, arUtilGetPixelSize(pixelFormat), arUtilGetPixelFormatName(pixelFormat));

    // Tell arVideo what the typical focal distance will be. Note that this does NOT
    // change the actual focus, but on devices with non-fixed focus, it lets arVideo
    // choose a better set of camera parameters.
    ar2VideoSetParami(m_vid, AR_VIDEO_PARAM_AVFOUNDATION_FOCUS_PRESET, AR_VIDEO_AVFOUNDATION_FOCUS_0_3M); // Default is 0.3 metres. See <AR/video.h> for allowable values.

    if (m_noCpara) return open3(NULL);

    // Load the camera parameters, resize for the window and init.
    // User-supplied camera parameters take precedence, then internal, then fetched. Otherwise, a default will be created.
    ARParam cparam;
    if (cameraParamBuffer) {
        if (arParamLoadFromBuffer(cameraParamBuffer, cameraParamBufferLen, &cparam) < 0) {
            ARLOGe("Error: failed to load camera parameters from buffer.\n");
            this->close();
            return false;
        } else {
            ARLOGi("Camera parameters loaded from buffer.\n");
            return (open3(&cparam));
        }
    } else if (cameraParam && *cameraParam) {
        if (arParamLoad(cameraParam, 1, &cparam) < 0) {
            ARLOGe("Error: failed to load camera parameters from file '%s'.\n", cameraParam);
            this->close();
            return false;
        } else {
            ARLOGi("Camera parameters loaded from file '%s'.\n", cameraParam);
            return (open3(&cparam));
        }
    } else {
        // Look for internal camera parameters.
        if (ar2VideoGetCParam(m_vid, &cparam) == 0) {
            ARLOGi("Using internal camera parameters.\n");
            return (open3(&cparam));
        } else {
            // Search database.
            if (ar2VideoGetCParamAsync(m_vid, open2CparamCallback, (void *)this) == 0) {
                ARLOGi("Fetching external camera parameters.\n");
                return true;
            }
        }
        return open3(NULL);
    }
}

// static callback method.
void WebARKitVideoSource::open2CparamCallback(const ARParam *cparam_p, void *userdata)
{
    if (!userdata) {
        ARLOGe("WebARKitVideoSource::open2CparamCallback: userdata was NULL.\n");
        return;
    }
    WebARKitVideoSource *vs = reinterpret_cast<WebARKitVideoSource *>(userdata);
    vs->open3(cparam_p);
}

bool WebARKitVideoSource::open3(const ARParam *cparam_p)
{
    if (!m_noCpara) {
        ARParam cparam;

        if (cparam_p) {
            cparam = *cparam_p;
        } else {
            arParamClearWithFOVy(&cparam, videoWidth, videoHeight, M_PI_4); // M_PI_4 radians = 45 degrees.
            ARLOGw("Using default camera parameters for %dx%d image size, 45 degrees vertical field-of-view.\n", videoWidth, videoHeight);
        }

        if (cparam.xsize != videoWidth || cparam.ysize != videoHeight) {
#ifdef DEBUG
            ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam.xsize, cparam.ysize);
#endif
            arParamChangeSize(&cparam, videoWidth, videoHeight, &cparam);
        }
        if (!(cparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET))) {
            ARLOGe("Error: failed to create camera parameters lookup table.\n");
            this->close();
            return false;
        }
    }

    int err = ar2VideoCapStart(m_vid);
    if (err != 0) {
        if (err == -2) {
            ARLOGe("can't start video, device unavailable.\n");
            setError(WEBARKIT_ERROR_DEVICE_UNAVAILABLE);
        } else {
            ARLOGe("returned error code \"%d\" starting video capture.\n", err);
        }
        this->close();
        return false;
    }

    deviceState = DEVICE_RUNNING;

    ARLOGd("Video capture started.\n");
    return true;
}

bool WebARKitVideoSource::captureFrame()
{
    if (deviceState == DEVICE_RUNNING) {
        if (m_captureFrameWaitCount) {
            ARLOGi("Video source is running. (Waited %d calls.)\n", m_captureFrameWaitCount);
            m_captureFrameWaitCount = 0;
        }
        pthread_rwlock_wrlock(&m_frameBufferLock);
        AR2VideoBufferT *vbuff = ar2VideoGetImage(m_vid);
        pthread_rwlock_unlock(&m_frameBufferLock);
        if (vbuff && vbuff->fillFlag) {
            m_frameBuffer = vbuff;
            return true;
        }
    } else {
        if (!m_captureFrameWaitCount) {
            ARLOGi("Waiting for video source.\n");
        }
        m_captureFrameWaitCount++;
    }
    return false;
}

bool WebARKitVideoSource::close()
{
    ARLOGd("WebARKitVideoSource::close(): called.\n");

    if (deviceState == DEVICE_CLOSED) {
        ARLOGd("WebARKitVideoSource::close(): already closed.\n");
        return true;
    }

    if (deviceState == DEVICE_RUNNING) {
        ARLOGd("WebARKitVideoSource::close(): stopping video.\n");

        pthread_rwlock_wrlock(&m_frameBufferLock);
        int err = ar2VideoCapStop(m_vid);
        m_frameBuffer = NULL;
        pthread_rwlock_unlock(&m_frameBufferLock);
        if (err != 0)
            ARLOGe("Error \"%d\" stopping video.\n", err);

        if (cparamLT) arParamLTFree(&cparamLT);

        deviceState = DEVICE_OPEN;
    }

    ARLOGi("Closing webarkit video.\n");
    if (ar2VideoClose(m_vid) != 0)
        ARLOGe("Error closing video.\n");

    m_vid = NULL;
    deviceState = DEVICE_CLOSED; // webarkit video source is always ready to be opened.

    return true;
}

void WebARKitVideoSource::setError(int error)
{
    if (m_error == WEBARKIT_ERROR_NONE) {
        m_error = error;
    }
}

int WebARKitVideoSource::getError()
{
    int temp = m_error;
    if (temp != WEBARKIT_ERROR_NONE) {
        m_error = WEBARKIT_ERROR_NONE;
    }
    return temp;
}

bool WebARKitVideoSource::isOpen() const
{
    return deviceState != DEVICE_CLOSED;
}

bool WebARKitVideoSource::isRunning() const
{
    return deviceState == DEVICE_RUNNING;
}

ARParamLT* WebARKitVideoSource::getCameraParameters() const
{
    return cparamLT;
}

int WebARKitVideoSource::getVideoWidth() const
{
    return videoWidth;
}

int WebARKitVideoSource::getVideoHeight() const
{
    return videoHeight;
}

AR_PIXEL_FORMAT WebARKitVideoSource::getPixelFormat() const
{
    return pixelFormat;
}

AR2VideoBufferT* WebARKitVideoSource::checkoutFrameIfNewerThan(const AR2VideoTimestampT time)
{
    pthread_rwlock_rdlock(&m_frameBufferLock);
    if (m_frameBuffer) {
        //ARLOGd("WebARKitVideoSource::checkoutFrameIfNewerThan(%" PRIu64 ", %" PRIu32 ") frame is available with time (%" PRIu64 ", %" PRIu32 ").\n", time.sec, time.usec, m_frameBuffer->time.sec, m_frameBuffer->time.usec);
        if  (m_frameBuffer->time.sec > time.sec || (m_frameBuffer->time.sec == time.sec && m_frameBuffer->time.usec > time.usec)) {
            return m_frameBuffer;
        }
    }
    pthread_rwlock_unlock(&m_frameBufferLock);
    return NULL;
}

void WebARKitVideoSource::checkinFrame(void)
{
    pthread_rwlock_unlock(&m_frameBufferLock);
}

AR2VideoParamT *WebARKitVideoSource::getAR2VideoParam(void)
{
    return m_vid;
}

bool WebARKitVideoSource::getFrameTextureRGBA32(uint32_t *buffer) {

    if (!buffer) return false; // Sanity check.

    AR2VideoBufferT *buff = checkoutFrameIfNewerThan(m_getFrameTextureTime);
    if (!buff) return false; // Check that a frame is actually available, and don't update the array if the current frame is the same is previous one.
    m_getFrameTextureTime = buff->time;

    int ret = videoRGBA(buffer, buff, videoWidth, videoHeight, pixelFormat);
    checkinFrame();
    if (ret < 0) {
        ARLOGe("WebARKitVideoSource::getFrameTextureRGBA32: videoRGBA error.\n");
        return false;
    }

    return true;
}

#if ARX_TARGET_PLATFORM_EMSCRIPTEN
int WebARKitVideoSource::webVideoPushInit(int width, int height, const char *pixelFormat, int camera_index, int camera_face)
{
    ARLOGi("WebARKitVideoSource::webVideoPushInit ... \n");

    if (deviceState == DEVICE_GETTING_READY) return 0; // This path will be exercised if another frame arrives while we're waiting for the callback.
    else if (deviceState != DEVICE_OPEN) {
        ARLOGe("WebARKitVideoSource::webVideoPushInit: Error: device not open.\n");
        return -1;
    }
    deviceState = DEVICE_GETTING_READY;

    int ret = (ar2VideoPushInit(m_vid, width, height, pixelFormat, camera_index, camera_face));
    deviceState = DEVICE_RUNNING; // after ar2VideoPushInit everything is ready to copy the video frames from JS into the HEAP to make it available for updateAR
    this->pixelFormat = ar2VideoGetPixelFormat(m_vid);
    if (this->pixelFormat < 0 ) {
        ARLOGe("Error: unable to get pixel format.\n");
        this->close();
        return false;
    }
    return ret;
}
#endif