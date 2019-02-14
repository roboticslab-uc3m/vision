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
#include <opencv2/opencv.hpp>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
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

// Variables

int yarpserver_ok=0;
string source_video="/home/tiagoentrenamiento/Vídeos/tiago.mp4";// Test: /home/tiagoentrenamiento/Vídeos/tiago.mp4
string labels = "models/ssd_mobilenet_v1_egohands/labels_map.pbtxt";
string graph = "models/ssd_mobilenet_v1_egohands/frozen_inference_graph.pb";

int main(){

  // Welcome message
  std::system("clear");
  cout<<"**************************************************************************"<<endl;
  cout<<"**************************************************************************"<<endl;
  cout<<"                     Program: Tensorflow Detector 2D                      "<<endl;
  cout<<"                     Author: David Velasco García                         "<<endl;
  cout<<"                             @davidvelascogarcia                          "<<endl;
  cout<<"**************************************************************************"<<endl;
  cout<<"**************************************************************************"<<endl;
  Time::delay(1);
  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"System date:"<<endl;
  std::system("date");
  Time::delay(1);
  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Starting system..."<<endl;
  Time::delay(1);
  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Welcome ..."<<endl;
  std::system("whoami");
  Time::delay(1);
  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Initializing ..."<<endl;
  Time::delay(1);
  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Loading TensorFlow 2D detector module..."<<endl;
  Time::delay(1);

  //Red yarp
  Network yarp;

  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Initializing YARP network..."<<endl;
  Time::delay(1);

  // Apertura puerto emisión
  Port sender_port_pre;
  Port sender_port_post;
  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Opening sender ports..."<<endl;
  Time::delay(1);
  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Opening pre-processed video port with the name /video_sender_pre."<<endl;
  Time::delay(1);
  sender_port_pre.open("/video_sender_pre");
  Time::delay(1);
  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Opening post-processed port with the name /video_sender_post."<<endl;
  Time::delay(1);
  sender_port_post.open("/video_sender_post");
  Time::delay(1);

  // Comprobación yarpserver
  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Checking yarpserver status..."<<endl;
  Time::delay(1);
  while(yarpserver_ok==0){

  if (!yarp::os::Network::checkNetwork())
  {

      cout<<endl;
      cout<<endl;
      cout<<"YARPSERVER status: FAIL"<<endl;
      cout<<"Please star yarpserver or connect to yarpserver already running..."<<endl;

      Time::delay(1);

  }else{
      std::system("clear");
      cout<<endl;
      cout<<endl;
      cout<<"YARPSERVER status: OK"<<endl;
      cout<<endl;
      cout<<endl;
      yarpserver_ok=1;
      Time::delay(1);
  }
  }

  std::system("clear");
  // Test sesión tensorflow
  tensorflow_test test;
  test.run();

  std::system("clear");
  // Instanciar detector
  tensorflowDetection2D detector;

  // Inicializar
  detector.init(source_video, labels, graph);
  detector.detector(sender_port_pre, sender_port_post);

  std::system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Closing Tensorflow 2D detector module..."<<endl;
  Time::delay(5);

  return 0;
}
