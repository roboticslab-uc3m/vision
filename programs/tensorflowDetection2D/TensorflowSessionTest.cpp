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
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/Time.h>
#include <opencv2/opencv.hpp>
#include <yarp/sig/Image.h>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>
#include "TensorflowDetection2D.hpp"
#include "TensorflowSessionTest.h"

// Namespace

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace cv;
using namespace std;
using namespace tensorflow;


tensorflow_test::tensorflow_test()
{

}

// Tensorflow session test

void tensorflow_test::run()
{
    std::system("clear");
    cout<<endl;
    cout<<endl;
    cout<<"Initializing tensorflow test session..."<<endl;
    Time::delay(1);
    std::system("clear");
    cout<<endl;
    cout<<endl;
    Session* session;
    Status status = NewSession(SessionOptions(), &session);
    if (!status.ok()) {
            cout << status.ToString() << "\n";

    }
    Time::delay(1);
    std::system("clear");
    cout<<endl;
    cout<<endl;
    cout << "Tensorflow test session created correctly."<<endl;
    Time::delay(1);

}
