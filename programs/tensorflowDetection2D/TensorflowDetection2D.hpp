#ifndef TENSORFLOWDETECTION2D_HPP
#define TENSORFLOWDETECTION2D_HPP

/*
 * ************************************************************
 *      Program: Tensorflow Detection 2D Module
 *      Type: tensorflowdetection2d.hpp
 *      Author: David Velasco Garcia @davidvelascogarcia
 * ************************************************************
 */

// Librerias

#include <iostream>
#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Image.h>
#include <opencv2/opencv.hpp>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>


// Espacios de nombres

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace cv;
using namespace std;
using namespace tensorflow;

class tensorflowDetection2D
{
public:
    tensorflowDetection2D();
    void init(string source_video, string labels, string graph);
    Mat get_image();
    void send_post(Mat img_post, Port puerto_post);
    void send_pre(Mat img_pre, Port puerto_pre);
    int detector(Port puerto_pre, Port puerto_post);

// Variables

    string cam_path="0";
    // Test tiagoentrenamiento: /home/tiagoentrenamiento/Vídeos/tiago.mp4
    // Test tiagoentrenamiento: /home/tiagoentrenamiento/Imágenes/tiago.jpeg

private:
    string video_source;
    string vgg16_graph;
    string vgg16_labels;

};

#endif // TENSORFLOWDETECTION2D_HPP
