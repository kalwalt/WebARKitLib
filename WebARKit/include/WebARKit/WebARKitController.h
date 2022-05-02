/*
 *  WebARKitController.h
 *  artoolkitX
 *
 *  A C++ class encapsulating core controller functionality of artoolkitX.
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


#ifndef WEBARKITCONTROLLER_H
#define WEBARKITCONTROLLER_H

#include <WebARKit/Platform.h>
#include <ARX/AR/ar.h>
#include <WebARKit/WebARKitVideoSource.h>
#include <WebARKit/WebARKitVideoView.h>
#include <WebARKit/WebARKitTrackerSquare.h>
#if HAVE_NFT
#  include <WebARKit/WebARKitTrackerNFT.h>
#endif
#if HAVE_2D
#  include <WebARKit/WebARKitTracker2d.h>
#  include <WebARKit/WebARKitTrackerOrb2d.h>
#endif
#include <WebARKit/WebARKitTrackable.h>


#include <vector>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#if !defined(_WINRT)
#  include <pthread.h>
#else
#  define pthread_mutex_t               CRITICAL_SECTION
#  define pthread_mutex_init(pm, a)     InitializeCriticalSectionEx(pm, 4000, CRITICAL_SECTION_NO_DEBUG_INFO)
#  define pthread_mutex_lock(pm)        EnterCriticalSection(pm)
#  define pthread_mutex_unlock(pm)      LeaveCriticalSection(pm)
#  define pthread_mutex_destroy(pm)     DeleteCriticalSection(pm)
#endif

/**
 * Wrapper for artoolkitX functionality. This class handles artoolkitX initialisation, updates,
 * and cleanup. It maintains a collection of trackables, providing methods to add and remove them.
 */
class WebARKitController {

private:
#pragma mark Private types and instance variables
    // ------------------------------------------------------------------------------
    // Private types and instance variables.
    // ------------------------------------------------------------------------------

	typedef enum {
		NOTHING_INITIALISED,			///< No initialisation yet and no resources allocated.
		BASE_INITIALISED,				///< Trackable management initialised, trackables can be added.
		WAITING_FOR_VIDEO,				///< Waiting for video source to become ready.
		DETECTION_RUNNING				///< Video running, additional initialisation occurred, tracking running.
	} WebARToolKitState;

	WebARToolKitState state;				///< Current state of operation, progress through initialisation
    bool stateWaitingMessageLogged;

	char* versionString;				///< artoolkitX version string

	WebARKitVideoSource *m_videoSource0;      ///< VideoSource providing video frames for tracking
	WebARKitVideoSource *m_videoSource1;      ///< VideoSource providing video frames for tracking
    ARdouble m_transL2R[3][4];
    bool m_videoSourceIsStereo;
    AR2VideoTimestampT m_updateFrameStamp0;
    AR2VideoTimestampT m_updateFrameStamp1;
    WebARKitVideoView *m_arVideoViews[2];

    std::vector<WebARKitTrackable *> m_trackables;    ///< List of trackables.

    bool doSquareMarkerDetection;
    std::shared_ptr<WebARKitTrackerSquare> m_squareTracker;
#if HAVE_NFT
    bool doNFTMarkerDetection;
    std::shared_ptr<WebARKitTrackerNFT> m_nftTracker;
#endif
#if HAVE_2D
    bool doTwoDMarkerDetection;
		bool doOrbTwoDMarkerDetection;
    std::shared_ptr<WebARKitTracker2d> m_twoDTracker;
    std::shared_ptr<WebARKitTrackerOrb2d> m_OrbTwoDTracker;
#endif
    int m_error;
    void setError(int error);

#pragma mark Private methods.
    // ------------------------------------------------------------------------------
    // Private methods.
    // ------------------------------------------------------------------------------

    //
    // Internal trackable management.
    //

	/**
	 * Adds a trackable to the collection.
	 * @param trackable	The trackable to add
	 * @return			true if the trackable was added successfully, otherwise false
	 */
	bool addTrackable(WebARKitTrackable* trackable);

	/**
	 * Removes the specified trackable.
	 * @param trackable		The trackable to remove
	 * @return				true if the trackable was removed, false if an error occurred.
	 */
	bool removeTrackable(WebARKitTrackable* trackable);


public:
#pragma mark Public API
    // ------------------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------------------

    /**
	 * Constructor.
	 */
	WebARKitController();

	/**
	 * Destructor.
	 */
	~WebARKitController();

	/**
	 * Returns a string containing the artoolkitX version, such as "10.0.0".
	 * @return		The artoolkitX version
	 */
	const char* getARToolKitVersion();

    int getError();

	/**
	 * Start trackable management so trackables can be added and removed.
     * @return       true if initialisation was OK, false if an error occured.
	 */
	bool initialiseBase();

    std::shared_ptr<WebARKitTrackerSquare> getSquareTracker() { return m_squareTracker; };
#if HAVE_NFT
    std::shared_ptr<WebARKitTrackerNFT> getNFTTracker() { return m_nftTracker; };
#endif
#if HAVE_2D
    std::shared_ptr<WebARKitTracker2d> get2dTracker() { return m_twoDTracker; };
		std::shared_ptr<WebARKitTrackerOrb2d> getOrb2dTracker() { return m_OrbTwoDTracker; };
#endif

	/**
	 * Report whether artoolkit was initialized and a trackable can be added.
     * Trackables can be added once basic initialisation has occurred.
	 * @return  true if adding a trackable is currently possible
	 */
	bool isInited();

	/**
	 * Start video capture and tracking. (AR/NFT initialisation will begin on a subsequent call to update().)
	 * @param vconf			Video configuration string.
	 * @param cparaName		Camera parameters filename, or NULL if camera parameters file not being used.
	 * @param cparaBuff		A byte-buffer containing contents of a camera parameters file, or NULL if a camera parameters file is being used.
	 * @param cparaBuffLen	Length (in bytes) of cparaBuffLen, or 0 if a camera parameters file is being used.
	 * @return				true if video capture and tracking was started, otherwise false.
	 */
	bool startRunning(const char* vconf, const char* cparaName, const char* cparaBuff, const long cparaBuffLen);

	/**
	 * Start stereo video capture and tracking. (AR/NFT initialisation will begin on a subsequent call to update().)
	 * @param vconfL		Video configuration string for the "left" video source.
	 * @param cparaNameL	Camera parameters filename for the "left" video source, or NULL if camera parameters file not being used.
	 * @param cparaBuffL	A byte-buffer containing contents of a camera parameters file for the "left" video source, or NULL if a camera parameters file is being used.
	 * @param cparaBuffLenL	Length (in bytes) of cparaBuffLenL, or 0 if a camera parameters file is being used.
	 * @param vconfR		Video configuration string for the "right" video source.
	 * @param cparaNameR	Camera parameters filename for the "right" video source, or NULL if camera parameters file not being used.
	 * @param cparaBuffR	A byte-buffer containing contents of a camera parameters file for the "right" video source, or NULL if a camera parameters file is being used.
	 * @param cparaBuffLenR	Length (in bytes) of cparaBuffLenR, or 0 if a camera parameters file is being used.
     * @param transL2RName	Stereo calibration filename, or NULL if stereo calibration file not being used.
     * @param transL2RBuff	A byte-buffer containing contents of a stereo calibration file, or NULL if a stereo calibration file is being used.
     * @param transL2RBuffLen Length (in bytes) of transL2RBuff, or 0 if a stereo calibration file is being used.
	 * @return				true if video capture and tracking was started, otherwise false.
	 */
	bool startRunningStereo(const char* vconfL, const char* cparaNameL, const char* cparaBuffL, const long cparaBuffLenL,
                            const char* vconfR, const char* cparaNameR, const char* cparaBuffR, const long cparaBuffLenR,
                            const char* transL2RName, const char* transL2RBuff, const long transL2RBuffLen);

	/**
	 * Reports width, height and pixel format of a video source.
     * To retrieve the size (in bytes) of each pixel, use arUtilGetPixelSize(*pixelFormat);
     * To get a C-string with the name of the pixel format, use arUtilGetPixelFormatName(*pixelFormat);
	 * @param videoSourceIndex Index into an array of video sources, specifying which source should be queried.
     * @param width Pointer to an int which will be filled with the width (in pixels) of the video frame, or NULL if this information is not required.
     * @param height Pointer to an int which will be filled with the height (in pixels) of the video frame, or NULL if this information is not required.
     * @param pixelFormat Pointer to an AR_PIXEL_FORMAT which will be filled with the pixel format of the video frame, or NULL if this information is not required.
	 * @return		true if the video source(s) is/are open and returning frames, otherwise false.
	 */
    bool videoParameters(const int videoSourceIndex, int *width, int *height, AR_PIXEL_FORMAT *pixelFormat);

	/**
	 * Returns true if video capture and tracking is running.
	 * @return		true if the video source(s) is/are open and returning frames, otherwise false.
	 */
	bool isRunning();

    /**
	 * Video capture and tracking stops, but trackables are still valid and can be configured.
	 * @return				true if video capture and tracking was stopped, otherwise false.
	 */
	bool stopRunning();

	/**
	 * Stop, if running. Remove all trackables, clean up all memory.
     * Starting again from this state requires initialiseBase() to be called again.
	 * @return				true if shutdown was successful, otherwise false
	 */
	bool shutdown();

    /**
     * Populates the provided array with the ARToolKit projection matrix. The projection matrix is computed
     * once the video source has been opened, and camera parameters become available. If this method is called
     * before this happens, then the passed array is untouched and the method will return false.
     * @param videoSourceIndex Index into an array of video sources, specifying which source should be queried.
     * @param projectionNearPlane Near plane distance for projection matrix calculation.
     * @param projectionFarPlane Far plane distance for projection matrix calculation.
     * @param proj        Array to populate with OpenGL compatible projection matrix
     * @return            true if the projection matrix has been computed, otherwise false
     */
    bool projectionMatrix(const int videoSourceIndex, const ARdouble projectionNearPlane, const ARdouble projectionFarPlane, ARdouble proj[16]);

    bool drawVideoInit(const int videoSourceIndex);

    bool drawVideoSettings(const int videoSourceIndex, const int width, const int height, const bool rotate90, const bool flipH, const bool flipV, const WebARKitVideoView::HorizontalAlignment hAlign, const WebARKitVideoView::VerticalAlignment vAlign, const WebARKitVideoView::ScalingMode scalingMode, int32_t viewport[4]);

    bool drawVideo(const int videoSourceIndex);

    bool drawVideoFinal(const int videoSourceIndex);

	/**
	 * Adds a trackable as specified in the given configuration string. The format of the string can be
	 * one of:
	 *
     * - Square marker from pattern file: "single;pattern_file;pattern_width", e.g. "single;data/hiro.patt;80"
     * - Square marker from pattern passed in config: "single_buffer;pattern_width;buffer=[]", e.g. "single_buffer;80;buffer=234 221 237..."
     * - Square barcode marker: "single_barcode;barcode_id;pattern_width", e.g. "single_barcode;0;80"
     * - Multi-square marker: "multi;config_file", e.g. "multi;data/multi/marker.dat"
     * - Multi-square auto marker: "multi;origin_barcode_id;pattern_width", e.g. "multi;0;80.0"
     * - NFT marker: "nft;nft_dataset_pathname", e.g. "nft;gibraltar"
     *
	 * @param cfgs		The configuration string
	 * @return			The UID of the trackable instantiated based on the configuration string, or -1 if an error occurred.
	 */
	int addTrackable(const std::string& cfgs);

	/**
	 * Removes the trackable with the given ID.
	 * @param UID			The UID of the trackable to remove
	 * @return				true if the trackable was removed, false if an error occurred.
	 */
	bool removeTrackable(int UID);

	/**
	 * Clears the collection of trackables.
	 * @return				The number of trackables removed
	 */
	int removeAllTrackables();

	/**
	 * Returns the number of currently loaded trackables.
	 * @return				The number of currently loaded trackables.
	 */
	unsigned int countTrackables() const;

    /**
     * Returns the number of currently loaded trackables of the specified type.
     * @return                The number of currently loaded trackables.
     */
    unsigned int countTrackables(WebARKitTrackable::TrackableType trackableType) const;

    /**
     * Returns the trackable at the specified index.
     * @param index           0-based index into the array of trackables.
     * @return                The trackable, or NULL if no trackable exists at that index.
     */
    WebARKitTrackable* getTrackableAtIndex(unsigned int index);

    /**
     * Searches the collection of trackables for the given ID.
     * @param UID             The UID of the trackables to find
     * @return                The found trackable, or NULL if no matching UID was found.
     */
    WebARKitTrackable* findTrackable(int UID);

    /**
     * Requests the capture of a new frame from the video source(s).
     * In the case of stereo video sources, capture from both sources will be attempted.
     * @return                The capture succeeded, or false if no frame was captured.
     */
    bool capture();

    /**
     * Asks the video source to push the most recent frame into the passed-in buffer.
     * @param videoSourceIndex Index into an array of video sources, specifying which source should
     *      be queried.
     * @param buffer Pointer to a buffer of pixels (of type 'uint32_t') to be filled. It is the
     *      caller's responsibility to ensure that the buffer is of sufficient size. The pixels are
     *      RGBA in little-endian systems, or ABGR in big-endian systems.
     */
    bool updateTextureRGBA32(const int videoSourceIndex, uint32_t *buffer);

	/**
	 * Performs tracking and updates all trackables. The latest frame from the current
	 * video source is retrieved and analysed. Each trackable in the collection is updated with
	 * new tracking information. The trackable info array is
	 * iterated over, and detected trackables are matched up with those in the trackable collection. Each matched
	 * trackable is updated with visibility and transformation information. Any trackables not detected are considered
	 * not currently visible.
     *
	 * @return				true if update completed successfully, false if an error occurred
	 */
	bool update();

    /**
     * Populates the provided buffer with the current contents of the debug image.
     * @param videoSourceIndex Index into an array of video sources, specifying which source should
     *      be queried.
     * @param buffer Pointer to a buffer of pixels (of type 'uint32_t') to be filled. It is the
     *      caller's responsibility to ensure that the buffer is of sufficient size. The pixels are
     *      RGBA in little-endian systems, or ABGR in big-endian systems.
     */
    bool updateDebugTexture32(const int videoSourceIndex, uint32_t* buffer);

	/**
	 * Populates the provided color buffer with the image for the specified pattern.
	 * @param	patternID	The artoolkitX pattern ID to use
	 * @param	buffer		The color buffer to populate
	 * @return				true if successful, false if an error occurred
	 */
	bool getPatternImage(int patternID, uint32_t* buffer);

	/**
	 * Loads an optical parameters structure from file or from buffer.
     *
     * @param optical_param_name If supplied, points to a buffer specifying the path
     *      to the optical parameters file (as generated by the calib_optical utility.)
     * @param optical_param_buff If optical_param_name is NULL, the contents of this
     *      buffer will be interpreted as containing the contents of an optical
     *      parameters file.
     * @param optical_param_buffLen Length of the buffer specified in optical_param_buff.
     *      Ignored if optical_param_buff is NULL.
     * @param projectionNearPlane If p is non-NULL, near plane distance for projection matrix calculation.
     * @param projectionFarPlane If p is non-NULL, far plane distance for projection matrix calculation.
     * @param fovy_p Pointer to an ARdouble, which will be filled with the
     *      field-of-view (Y axis) component of the optical parameters.
     * @param aspect_p Pointer to an ARdouble, which will be filled with the
     *      aspect ratio (width / height) component of the optical parameters.
     * @param m Pointer to an array of 16 ARdoubles, which will be filled with the
     *      transformation matrix component of the optical parameters.
     * @param p (Optional) May be NULL, or a pointer to an array of 16 ARdoubles,
     *      which will be filled with the perspective matrix calculated from fovy and aspect
     *      combined with the near and far projection values supplied in projectionNearPlane and
     *      projectionFarPlane.
     */
    bool loadOpticalParams(const char *optical_param_name, const char *optical_param_buff, const long optical_param_buffLen, const ARdouble projectionNearPlane, const ARdouble projectionFarPlane, ARdouble *fovy_p, ARdouble *aspect_p, ARdouble m[16], ARdouble p[16]);


#if HAVE_2D
    /**
     * Loads a 2d image database.
     *
     * @param databaseFileName to load the 2d image database

     */
    bool load2DTrackerImageDatabase(const char* databaseFileName);

    /**
     * Saves all loaded 2D trackable to a database.
     *
     * @param databaseFileName to save the 2d image database
     */
    bool save2DTrackerImageDatabase(const char* databaseFileName);
#endif // HAVE_2D

#if ARX_TARGET_PLATFORM_EMSCRIPTEN

    int webVideoPushInitC(int videoSourceIndex, int width, int height, const char *pixelFormat, int camera_index, int camera_face);

#endif

};


#endif // !WEBARKITCONTROLLER_H
