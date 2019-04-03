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
std::string labels;
std::string graph;

int main(int argc, char ** argv){

  // Welcome message
  std::cout<<"**************************************************************************"<<std::endl;
  std::cout<<"**************************************************************************"<<std::endl;
  std::cout<<"                     Program: Tensorflow Detector 2D                      "<<std::endl;
  std::cout<<"                     Author: David Velasco García                         "<<std::endl;
  std::cout<<"                             @davidvelascogarcia                          "<<std::endl;
  std::cout<<"**************************************************************************"<<std::endl;
  std::cout<<"**************************************************************************"<<std::endl;

  std::cout<<std::endl;
  std::cout<<"Starting system..."<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Welcome ..."<<std::endl;
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Initializing ..."<<std::endl;
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Loading TensorFlow 2D detector module..."<<std::endl;

  //Red yarp
  yarp::os::Network yarp;

  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Initializing YARP network..."<<std::endl;

  // Apertura puerto de recepción
  std::cout<<"Opening image input port with the name /tensorflowDetection2D/img:i."<<std::endl;
  yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inImg;
  inImg.open("/tensorflowDetection2D/img:i");

  // Apertura puerto emisión
  yarp::os::Port sender_port_pre;
  yarp::os::Port sender_port_post;
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Opening sender ports..."<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Opening post-processed port with the name /tensorflowDetection2D/img:o."<<std::endl;
  sender_port_post.open("/tensorflowDetection2D/img:o");

  // Comprobación yarpserver
  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Checking yarpserver status..."<<std::endl;
  while(yarpserver_ok==0){

  if (!yarp::os::Network::checkNetwork())
  {

      std::cout<<std::endl;
      std::cout<<std::endl;
      std::cout<<"YARPSERVER status: FAIL"<<std::endl;
      std::cout<<"Please star yarpserver or connect to yarpserver already running..."<<std::endl;


  }else{
      std::cout<<std::endl;
      std::cout<<std::endl;
      std::cout<<"YARPSERVER status: OK"<<std::endl;
      std::cout<<std::endl;
      std::cout<<std::endl;
      yarpserver_ok=1;
  }
  }

  std::cout<<"Locating pre-trained model and labels map..."<<std::endl;
  yarp::os::ResourceFinder rf;

  rf.setVerbose(true);
  rf.setDefaultContext("tensorflowDetection2D");
  rf.setDefaultConfigFile("tensorflowDetection2D.ini");
  rf.configure(argc, argv);
  std::string pathToModel = rf.check("pathToModel", yarp::os::Value(""), "documentation").asString();
  labels = rf.findFileByName("labels_map.pbtxt");
  graph = rf.findFileByName("frozen_inference_graph.pb");

  // Instanciar detector
  tensorflowDetection2D detector;

  // Inicializar
  detector.init(labels, graph);
  detector.detector(sender_port_post, &inImg);

  std::cout<<std::endl;
  std::cout<<std::endl;
  std::cout<<"Closing Tensorflow 2D detector module..."<<std::endl;

  return 0;
}
