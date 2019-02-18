/*
 * ************************************************************
 *      Program: Tensorflow Detection 2D Module
 *      Type: main.cpp
 *      Author: David Velasco Garcia @davidvelascogarcia
 * ************************************************************
 */

// Libraries

#include <iostream>
#include <cstdlib>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
#include <opencv2/opencv.hpp>
#include <yarp/sig/Image.h>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>
#include "TensorflowDetection2D.hpp"
#include "TensorflowSessionTest.h"


tensorflow_test::tensorflow_test()
{

}

// Tensorflow session test

void tensorflow_test::run()
{
    std::system("clear");
    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Initializing tensorflow test session..."<<std::endl;
    yarp::os::Time::delay(1);
    std::system("clear");
    std::cout<<std::endl;
    std::cout<<std::endl;
    tensorflow::Session* session;
    tensorflow::Status status = NewSession(tensorflow::SessionOptions(), &session);
    if (!status.ok()) {
            std::cout << status.ToString() << "\n";

    }
    yarp::os::Time::delay(1);
    std::system("clear");
    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout << "Tensorflow test session created correctly."<<std::endl;
    yarp::os::Time::delay(1);

}
