// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HAAR_DETECTION_2D_HPP__
#define __HAAR_DETECTION_2D_HPP__

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>

#include <yarp/os/Bottle.h>
#include <yarp/sig/ImageDraw.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "SegmentorThread.hpp" // MUST GO AWAY!!

#include <ColorDebug.h>

#include "Transformation.hpp"

namespace roboticslab
{

class HaarDetection2D
{
public:
    yarp::sig::ImageOf<yarp::sig::PixelRgb> run(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg, cv::CascadeClassifier object_cascade);
    cv::Mat inCvMatPost;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImgPre;
    cv::CascadeClassifier object_cascadeFile;
private:

};

}  // namespace roboticslab

#endif  // __HAAR_DETECTION_2D_HPP__
