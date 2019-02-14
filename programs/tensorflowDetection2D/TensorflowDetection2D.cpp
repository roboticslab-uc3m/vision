/*
 * ************************************************************
 *      Program: Tensorflow Detection 2D Module
 *      Type: tensorflowdetection2d.cpp
 *      Author: David Velasco Garcia @davidvelascogarcia
 * ************************************************************
 */

// Libraries

#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.hpp>
#include <time.h>
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/util/command_line_flags.h"
#include "TensorflowDetection2D.hpp"
#include "TensorflowSessionTest.h"
#include "TensorflowDetector.hpp"
#include "MainDetector.hpp"

using tensorflow::Flag;
using tensorflow::Tensor;
using tensorflow::Status;
using tensorflow::string;
using tensorflow::int32;


// Namespace

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace cv;
using namespace std;
using namespace tensorflow;



tensorflowDetection2D::tensorflowDetection2D()
{
}

void tensorflowDetection2D::init(string source_video, string labels, string graph)
{
  video_source=source_video;
  vgg16_graph=graph;
  vgg16_labels=labels;
}

Mat tensorflowDetection2D::get_image()
{
    VideoCapture webcam;
    Mat picture;
    // Abrir cámara
    int cam_ok=0;
    while(cam_ok==0){
    if(!webcam.open(cam_path)){
      cout << "I can´t open source video, check connection..." << endl;
    }else{
        cout<<"Source video opended correctly"<<endl;
        cam_ok=1;
    }}

    webcam >> picture;

    return picture;

}
int tensorflowDetection2D::detector(Port sender_port_pre,Port sender_port_post){

  maindetector detection_module;
  detection_module.detect(vgg16_labels, vgg16_graph, video_source, sender_port_pre, sender_port_post);
  return 0;
}


void tensorflowDetection2D::send_post(Mat img_post, Port sender_port_post)
{
  ImageOf<PixelBgr> B;
  B.setExternal(img_post.data,img_post.size[1],img_post.size[0]);
  sender_port_post.write(B);

}

void tensorflowDetection2D::send_pre(Mat img_pre, Port sender_port_pre)
{
  ImageOf<PixelBgr> B;
  B.setExternal(img_pre.data,img_pre.size[1],img_pre.size[0]);
  sender_port_pre.write(B);

}
