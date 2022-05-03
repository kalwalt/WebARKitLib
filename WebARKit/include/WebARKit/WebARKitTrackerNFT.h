/*
 *  WebARKitTrackernft.h
 *  artoolkitX
 *
 *  A C++ class implementing the artoolkitX NFT fiducial marker tracker.
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


#ifndef WEBARKITTRACKERNFT_H
#define WEBARKITTRACKERNFT_H

#include <ARX/AR/config.h>
#if HAVE_NFT

#include <WebARKit/WebARKitTrackableNFT.h>
#include <WebARKit/WebARKitTrackerVideo.h>
#include <ARX/AR2/tracking.h>
#include <ARX/KPM/kpm.h>

#define PAGES_MAX 64

class WebARKitTrackerNFT : public WebARKitTrackerVideo {
public:
    WebARKitTrackerNFT();
    ~WebARKitTrackerNFT();
    
    WebARKitTrackerType type() const override {
        return WebARKitTrackerType::TEXTURE2D_FIDUCIAL;
    }
    
    std::vector<std::string> trackableConfigurations() const override {
        std::vector<std::string> sv;
        sv.push_back("nft");
        return sv;
    }

    bool initialize() override;
    
    void setNFTMultiMode(bool on);
    bool NFTMultiMode() const;
    int detectNFTMarker(ARUint8 *imageLumaPtr);
    int getNFTMarker(int markerIndex, ARUint8 *imageLumaPtr);
    
    bool start(ARParamLT *paramLT, AR_PIXEL_FORMAT pixelFormat) override;
    bool start(ARParamLT *paramLT0, AR_PIXEL_FORMAT pixelFormat0, ARParamLT *paramLT1, AR_PIXEL_FORMAT pixelFormat1, const ARdouble transL2R[3][4]) override;
    bool isRunning() override;
    bool update(AR2VideoBufferT *buff, std::vector<WebARKitTrackable *>& trackables) override;
    bool update(AR2VideoBufferT *buff0, AR2VideoBufferT *buff1, std::vector<WebARKitTrackable *>& trackables) override;
    bool stop() override;
    void terminate() override;

    WebARKitTrackable *newTrackable(std::vector<std::string> config) override;
    void deleteTrackable(WebARKitTrackable **trackable_p) override;
    
private:
    bool m_videoSourceIsStereo;
    bool m_nftMultiMode;
    bool m_kpmRequired;
    bool m_kpmBusy;
    bool m_nft_loaded;
    int m_detectedPage;
    int m_surfaceSetCount;
    // NFT data.
    //THREAD_HANDLE_T     *trackingThreadHandle;
    bool     trackingThreadHandle;
    AR2HandleT          *m_ar2Handle;
    KpmHandle           *m_kpmHandle;
    AR2SurfaceSetT      *m_surfaceSet[PAGES_MAX]; // Weak-reference. Strong reference is now in WebARKitTrackableNFT class.
    ARdouble m_transL2R[3][4];          ///< For stereo tracking, transformation matrix from left camera to right camera.

    bool unloadNFTData();
    bool loadNFTData(std::vector<WebARKitTrackable *>& trackables);
};

#endif // HAVE_NFT
#endif // !WEBARKITTRACKERNFT_H
