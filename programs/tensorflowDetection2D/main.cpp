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
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <opencv2/opencv.hpp>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>
#include "TensorflowDetection2D.hpp"


// Variables

int yarpserver_ok=0;
std::string labels = "models/ssd_mobilenet_v1_egohands/labels_map.pbtxt";
std::string graph = "models/ssd_mobilenet_v1_egohands/frozen_inference_graph.pb";

int main(){

  // Welcome message
  std::system("clear");
  std::cout<<"**************************************************************************"<<std::endl;
  std::cout<<"**************************************************************************"<<std::endl;
  std::cout<<"                     Program: Tensorflow Detector 2D                      "<<std::endl;
  std::cout<<"                     Author: David Velasco García                         "<<std::endl;
  std::cout<<"                             @davidvelascogarcia                          "<<std::endl;
  std::cout<<"**************************************************************************"<<std::endl;
  std::cout<<"**************************************************************************"<<std::endl;
  yarp::os::Time::delay(1);
  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"System date:"<<std::endl;
  std::system("date");
  yarp::os::Time::delay(1);
  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Starting system..."<<std::endl;
  yarp::os::Time::delay(1);
  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Welcome ..."<<std::endl;
  std::system("whoami");
  yarp::os::Time::delay(1);
  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Initializing ..."<<std::endl;
  yarp::os::Time::delay(1);
  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Loading TensorFlow 2D detector module..."<<std::endl;
  yarp::os::Time::delay(1);

  //Red yarp
  yarp::os::Network yarp;

  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Initializing YARP network..."<<std::endl;
  yarp::os::Time::delay(1);

  // Apertura puerto de recepcións
  std::cout<<"Opening image input port with the name /tensorflowDetection2D/img:i."<<std::endl;
  yarp::os::Time::delay(1);
  //BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inImg;

  //inImg.open("/tensorflowDetection2D/img:i");
  yarp::os::Time::delay(1);
  std::system("clear");

  // Apertura puerto emisión
  yarp::os::Port sender_port_pre;
  yarp::os::Port sender_port_post;
  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Opening sender ports..."<<std::endl;
  yarp::os::Time::delay(1);
  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Opening pre-processed video port with the name /tensorflowDetection2D/img:pre."<<std::endl;
  yarp::os::Time::delay(1);
  sender_port_pre.open("/tensorflowDetection2D/img:pre");
  yarp::os::Time::delay(1);
  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Opening post-processed port with the name /tensorflowDetection2D/img:o."<<std::endl;
  yarp::os::Time::delay(1);
  sender_port_post.open("/tensorflowDetection2D/img:o");
  yarp::os::Time::delay(1);

  // Comprobación yarpserver
  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Checking yarpserver status..."<<std::endl;
  yarp::os::Time::delay(1);
  while(yarpserver_ok==0){

  if (!yarp::os::Network::checkNetwork())
  {

      std::cout<<std::endl;
      std::cout<<std::endl;
      std::cout<<"YARPSERVER status: FAIL"<<std::endl;
      std::cout<<"Please star yarpserver or connect to yarpserver already running..."<<std::endl;

      yarp::os::Time::delay(1);

  }else{
      std::system("clear");
      std::cout<<std::endl;
      std::cout<<std::endl;
      std::cout<<"YARPSERVER status: OK"<<std::endl;
      std::cout<<std::endl;
      std::cout<<std::endl;
      yarpserver_ok=1;
      yarp::os::Time::delay(1);
  }
  }

  std::system("clear");
  // Instanciar detector
  tensorflowDetection2D detector;

  // Inicializar
  detector.init(labels, graph);
  detector.detector(sender_port_pre, sender_port_post);

  std::system("clear");
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Closing Tensorflow 2D detector module..."<<std::endl;
  yarp::os::Time::delay(5);

  return 0;
}
