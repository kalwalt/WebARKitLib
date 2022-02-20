#ifndef __WEBARKIT_ORB_TRACKER_H__
#define __WEBARKIT_ORB_TRACKER_H__

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>

#define OUTPUT_SIZE     17

typedef struct {
    char valid;
    double *data;   // 9 elems in homography matrix + 8 elems in warped corners
} output_t;

int initAR(uchar refData[], size_t refCols, size_t refRows);
output_t *resetTracking(uchar imageData[], size_t cols, size_t rows);
output_t *track(uchar imageData[], size_t cols, size_t rows);

#endif