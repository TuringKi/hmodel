#ifndef _LIBSEG_H_
#define _LIBSEG_H_

#include "opencv/cv.h"

void hand_segmentation(cv::Mat& depth, cv::Mat& color, cv::Mat &sensor_silhouette);


#endif