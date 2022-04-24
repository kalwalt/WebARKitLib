/*
 *  WebARKitTrackableNFT.cpp
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
 *  Author(s): Philip Lamb.
 *
 */

#include <WebARKit/WebARKitTrackableNFT.h>


#if HAVE_NFT

WebARKitTrackableNFT::WebARKitTrackableNFT() : WebARKitTrackable(NFT),
    m_loaded(false),
    m_nftScale(1.0f),
    pageNo(-1),
    datasetPathname(NULL)
{
}

WebARKitTrackableNFT::~WebARKitTrackableNFT()
{
	if (m_loaded) unload();
}

bool WebARKitTrackableNFT::load(const char* dataSetPathname_in, KpmHandle* m_kpmHandle)
{
    if (m_loaded) unload();
    
	visible = visiblePrev = false;

    /*KpmRefDataSet *refDataSet = NULL;
    int pageCount = 0;

    // Load KPM data.
    KpmRefDataSet *refDataSet2;
    ARLOGi("Reading '%s.fset3'.\n", dataSetPathname_in);
    if (kpmLoadRefDataSet(dataSetPathname_in, "fset3", &refDataSet2) < 0) {
        ARLOGe("Error reading KPM data from '%s.fset3'.\n", dataSetPathname_in);
        pageNo = -1;
        //continue;
    }
    pageNo = pageCount;
    ARLOGi("  Assigned page no. %d.\n", pageCount);
    if (kpmChangePageNoOfRefDataSet(refDataSet2, KpmChangePageNoAllPages, pageCount) < 0) {
        ARLOGe("kpmChangePageNoOfRefDataSet\n");
        exit(-1);
    }
    if (kpmMergeRefDataSet(&refDataSet, &refDataSet2) < 0) {
        ARLOGe("kpmMergeRefDataSet\n");
        exit(-1);
    }
    ARLOGi("Done.\n");

    // For convenience, create a weak reference to the AR2 data.
    //m_surfaceSet[pageCount] = surfaceSet;
            
    pageCount++;
    if (pageCount == PAGES_MAX) {
        ARLOGe("Maximum number of NFT pages (%d) loaded.\n", PAGES_MAX);
        exit(-1);
    }
	
    // Load AR2 data.
    ARLOGi("Loading '%s.fset'.\n", dataSetPathname_in);
    if ((surfaceSet = ar2ReadSurfaceSet(dataSetPathname_in, "fset", NULL)) == NULL) {
        ARLOGe("Error reading data from '%s.fset'.\n", dataSetPathname_in);
        return (false);
    }
    ARLOGi("kpmHandle: %d.\n", m_kpmHandle);
    if (kpmSetRefDataSet(m_kpmHandle, refDataSet) < 0) {
        ARLOGe("kpmSetRefDataSet\n");
        exit(-1);
    }
    kpmDeleteRefDataSet(&refDataSet);*/
 	datasetPathname = strdup(dataSetPathname_in);
    
    allocatePatterns(1);

    patterns[0]->loadISet(surfaceSet->surface[0].imageSet, m_nftScale);
   
    m_loaded = true;
    
	return true;
}

bool WebARKitTrackableNFT::unload()
{
    if (m_loaded) {
        freePatterns();
        pageNo = -1;
        if (surfaceSet) {
            ARLOGi("Unloading '%s.fset'.\n", datasetPathname);
            ar2FreeSurfaceSet(&surfaceSet); // Sets surfaceSet to NULL.
        }
        if (datasetPathname) {
            free(datasetPathname);
            datasetPathname = NULL;
        }
        
        m_loaded = false;
    }
	return true;
}

bool WebARKitTrackableNFT::updateWithNFTResults(int detectedPage, float trackingTrans[3][4], ARdouble transL2R[3][4])
{
    if (!m_loaded) return false;
    
	visiblePrev = visible;
    
    // The marker will only have a pageNo if the data has actually been loaded by a call to ARController::loadNFTData().
	if (pageNo >= 0 && pageNo == detectedPage) {
        visible = true;
        for (int j = 0; j < 3; j++) {
            trans[j][0] = (ARdouble)trackingTrans[j][0];
            trans[j][1] = (ARdouble)trackingTrans[j][1];
            trans[j][2] = (ARdouble)trackingTrans[j][2];
            trans[j][3] = (ARdouble)(trackingTrans[j][3] * m_nftScale);
        }
	} else visible = false;

	return (WebARKitTrackable::update(transL2R)); // Parent class will finish update.
}

void WebARKitTrackableNFT::setNFTScale(const float scale)
{
    m_nftScale = scale;
    patterns[0]->loadISet(surfaceSet->surface[0].imageSet, m_nftScale);
}

float WebARKitTrackableNFT::NFTScale()
{
    return (m_nftScale);
}

#endif // HAVE_NFT
