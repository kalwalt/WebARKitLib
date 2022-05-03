/*
 *  trackingSubMod.c
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
 *  Author(s): Hirokazu Kato, Philip Lamb
 *
 */


#include "trackingSubMod.h"

#if HAVE_NFT

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    KpmHandle              *kpmHandle;      // KPM-related data.
    ARUint8                *imageLumaPtr;   // Pointer to image being tracked.
    int                     imageSize;      // Bytes per image.
    float                   trans[3][4];    // Transform containing pose of tracked image.
    int                     page;           // Assigned page number of tracked image.
    int                     flag;           // Tracked successfully.
} TrackingInitHandle;

static void *trackingInitMain();

TrackingInitHandle     *trackingInitHandle;

int trackingInitQuit()
{

    if (trackingInitHandle) {
        free( trackingInitHandle->imageLumaPtr );
        free( trackingInitHandle );
    }

    return 0;
}

bool trackingInitInit( KpmHandle *kpmHandle )
{

    if (!kpmHandle) {
        ARLOGe("trackingInitInit(): Error: NULL KpmHandle.\n");
        return false;
    }
    
    trackingInitHandle = (TrackingInitHandle *)malloc(sizeof(TrackingInitHandle));
    if( trackingInitHandle == NULL ) return false;
    trackingInitHandle->kpmHandle = kpmHandle;
    trackingInitHandle->imageSize = kpmHandleGetXSize(kpmHandle) * kpmHandleGetYSize(kpmHandle);
    trackingInitHandle->imageLumaPtr  = (ARUint8 *)malloc(trackingInitHandle->imageSize);
    trackingInitHandle->flag      = 0;

    return true;
}

int trackingInitStart( ARUint8 *imageLumaPtr )
{

    if (!imageLumaPtr) {
        ARLOGe("trackingInitStart(): Error: NULL imageLumaPtr.\n");
        return (-1);
    }
    
    if (!trackingInitHandle) {
        ARLOGe("trackingInitStart(): Error: NULL trackingInitHandle.\n");
        return (-1);
    }

    memcpy( trackingInitHandle->imageLumaPtr, imageLumaPtr, trackingInitHandle->imageSize );

    return 0;
}

int trackingInitGetResult( float trans[3][4], int *page )
{
    int  i, j;

    if (!trans || !page)  {
        ARLOGe("trackingInitGetResult(): Error: NULL trans or page.\n");
        return (-1);
    }

    
    if (!trackingInitHandle) return (-1);
    //trackingInitMain();
    if( trackingInitHandle->flag ) {
        for (j = 0; j < 3; j++) for (i = 0; i < 4; i++) trans[j][i] = trackingInitHandle->trans[j][i];
        *page = trackingInitHandle->page;
        return 1;
    }

    return -1;
}

static void *trackingInitMain()
{

    KpmHandle              *kpmHandle;
    KpmResult              *kpmResult = NULL;
    int                     kpmResultNum;
    ARUint8                *imageLumaPtr;
    float                  err;
    int                    i, j, k;

    if (!trackingInitHandle) {
        ARLOGe("Error starting tracking thread: empty trackingInitHandle.\n");
        return (NULL);
    }

    kpmHandle          = trackingInitHandle->kpmHandle;
    imageLumaPtr       = trackingInitHandle->imageLumaPtr;
    if (!kpmHandle || !imageLumaPtr) {
        ARLOGe("Error starting tracking thread: empty kpmHandle/imageLumaPtr.\n");
        return (NULL);
    }
    ARLOGi("Start tracking thread.\n");
    
    kpmGetResult( kpmHandle, &kpmResult, &kpmResultNum );

    for(;;) {

        kpmMatching(kpmHandle, imageLumaPtr);
        trackingInitHandle->flag = 0;
        for( i = 0; i < kpmResultNum; i++ ) {
            if( kpmResult[i].camPoseF != 0 ) continue;
            ARLOGi("kpmGetPose OK.\n");
            if( trackingInitHandle->flag == 0 || err > kpmResult[i].error ) { // Take the first or best result.
                trackingInitHandle->flag = 1;
                trackingInitHandle->page = kpmResult[i].pageNo;
                for (j = 0; j < 3; j++) for (k = 0; k < 4; k++) trackingInitHandle->trans[j][k] = kpmResult[i].camPose[j][k];
                err = kpmResult[i].error;
            }
        }
    }

    ARLOGi("End tracking thread.\n");
    return (NULL);
}

#endif // HAVE_NFT
