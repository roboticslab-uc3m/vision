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
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>
#include <opencv2/opencv.hpp>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>


class tensorflowDetection2D
{
public:
    tensorflowDetection2D();
    void init(std::string source_video, std::string labels, std::string graph);
    cv::Mat get_image();
    void send_post(cv::Mat img_post, yarp::os::Port sender_port_post);
    void send_pre(cv::Mat img_pre, yarp::os::Port sender_port_pre);
    int detector(yarp::os::Port sender_port_pre, yarp::os::Port sender_port_post);

// Variables

    std::string cam_path="0";
    // Test tiagoentrenamiento: /home/tiagoentrenamiento/Vídeos/tiago.mp4
    // Test tiagoentrenamiento: /home/tiagoentrenamiento/Imágenes/tiago.jpeg

private:
    std::string video_source;
    std::string vgg16_graph;
    std::string vgg16_labels;

};

#endif // TENSORFLOWDETECTION2D_HPP
