#ifndef TENSORFLOWDETECTION2D_HPP
#define TENSORFLOWDETECTION2D_HPP

/*
 * ************************************************************
 *      Program: Tensorflow Detection 2D Module
 *      Type: tensorflowdetection2d.hpp
 *      Author: David Velasco Garcia @davidvelascogarcia
 * ************************************************************
 */

// Libraries

#include <iostream>
#include <string>
#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <opencv2/opencv.hpp>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>


using namespace yarp::os;
using namespace yarp::sig;
class tensorflowDetection2D
{
public:
    tensorflowDetection2D();
    void init( std::string labels, std::string graph);
    cv::Mat get_image();
    int detector(yarp::os::Port sender_port_post, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *inImg);

// Variables

    std::string cam_path="0";

private:
    std::string vgg16_graph;
    std::string vgg16_labels;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *inImg_i;

};

#endif // TENSORFLOWDETECTION2D_HPP
