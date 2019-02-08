/*
 * ************************************************************
 *      Program: Tensorflow Detection 2D Module
 *      Type: main.cpp
 *      Author: David Velasco Garcia @davidvelascogarcia
 * ************************************************************
 */

// Librerias

#include <iostream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <opencv2/opencv.hpp>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>
#include "TensorflowDetection2D.hpp"
#include "TensorflowSessionTest.h"

// Espacios de nombres

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

  // Mensaje de bienvenida
  system("clear");
  cout<<"**************************************************************************"<<endl;
  cout<<"**************************************************************************"<<endl;
  cout<<"                     Program: Tensorflow Detector 2D                      "<<endl;
  cout<<"                     Author: David Velasco García                         "<<endl;
  cout<<"                             @davidvelascogarcia                          "<<endl;
  cout<<"**************************************************************************"<<endl;
  cout<<"**************************************************************************"<<endl;
  Time::delay(1);
  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Fecha del sistema:"<<endl;
  system("date");
  Time::delay(1);
  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Iniciando sistema..."<<endl;
  Time::delay(1);
  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Bienveni@"<<endl;
  system("whoami");
  Time::delay(1);
  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Iniciando ..."<<endl;
  Time::delay(1);
  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Arrancando módulo de detección TensorFlow 2D..."<<endl;
  Time::delay(1);

  //Red yarp
  Network yarp;

  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Iniciando red de YARP..."<<endl;
  Time::delay(1);

  // Apertura puerto emisión
  Port puerto_envio_pre;
  Port puerto_envio_post;
  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Iniciando puerto de envío de imagen..."<<endl;
  Time::delay(1);
  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Abriendo puerto de envío con nombre /emisor_video_pre."<<endl;
  Time::delay(1);
  puerto_envio_pre.open("/emisor_video_pre");
  Time::delay(1);
  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Abriendo puerto de envío con nombre /emisor_video_post."<<endl;
  Time::delay(1);
  puerto_envio_post.open("/emisor_video_post");
  Time::delay(1);

  // Comprobación yarpserver
  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Comprobando estado yarpserver..."<<endl;
  Time::delay(1);
  while(yarpserver_ok==0){

  if (!yarp::os::Network::checkNetwork())
  {

      cout<<endl;
      cout<<endl;
      cout<<"Servidor YARP: FAIL"<<endl;
      cout<<"Inicie un servidor yarp o conéctese a uno..."<<endl;

      Time::delay(1);

  }else{
      system("clear");
      cout<<endl;
      cout<<endl;
      cout<<"Servidor YARP: OK"<<endl;
      cout<<endl;
      cout<<endl;
      yarpserver_ok=1;
      Time::delay(1);
  }
  }

  system("clear");
  // Test sesión tensorflow
  tensorflow_test test;
  test.run();

  system("clear");
  // Instanciar detector
  tensorflowDetection2D detector;

  // Inicializar
  detector.init(source_video, labels, graph);
  detector.detector(puerto_envio_pre, puerto_envio_post);

  system("clear");
  cout<<endl;
  cout<<endl;
  cout<<"Cerrando módulo detector Tensorflow 2D..."<<endl;
  Time::delay(5);

  return 0;
}
