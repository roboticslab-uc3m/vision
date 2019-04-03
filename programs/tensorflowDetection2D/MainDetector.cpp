
// Libraries

#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <time.h>
#include <cstdlib>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.hpp>
#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"
#include "TensorflowDetector.hpp"
#include "MainDetector.hpp"
#include <map>

// Librerias

#include <iostream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

int maindetector::detect(std::string labels, std::string graph, yarp::os::Port sender_port_pre, yarp::os::Port sender_port_post, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *inImg) {



    // Path
    std::string LABELS = labels;
    std::string GRAPH = graph;

    // Set  nombres nodos entrada y salida
    tensorflow::string inputLayer = "image_tensor:0";
    std::vector<std::string> outputLayer = {"detection_boxes:0", "detection_scores:0", "detection_classes:0", "num_detections:0"};

    // Load .pb frozen model
    std::unique_ptr<tensorflow::Session> session;
    tensorflow::string graphPath = GRAPH;
    std::cout<<"The graph it´s going to be loaded:" << graphPath<<"."<<std::endl;
    std::cout<<"Loading graph..."<<std::endl;
    yarp::os::Time::delay(1);
    //LOG(INFO) << "Loading graph:" << graphPath<<" ...";

    tensorflow::Status loadGraphStatus = loadGraph(graphPath, &session);
    if (!loadGraphStatus.ok()) {
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<"Fail loading graph "<<graphPath<<"."<<std::endl;
        //LOG(ERROR) << "Loading graph: FAIL" << loadGraphStatus;
        yarp::os::Time::delay(1);
        return -1;
    } else
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<"Graph "<<graphPath<<" loaded correctly."<<std::endl;
        //LOG(INFO) << "Loading  graph: OK" << std::endl;
        yarp::os::Time::delay(1);


    // Cargar etiquetas
    std::map<int, std::string> labelsMap = std::map<int,std::string>();
    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Labels "<<LABELS<<" are going to be loaded."<<std::endl;
    yarp::os::Time::delay(1);
    tensorflow::Status readLabelsMapStatus = readLabelsMapFile(LABELS, labelsMap);
    if (!readLabelsMapStatus.ok()) {
        //LOG(ERROR) << "readLabelsMapFile(): ERROR" << loadGraphStatus;
        std::cout<<"Fail loading labels "<<LABELS<<"."<<std::endl;
        //LOG(INFO) << "Carga del graph: OK" << std::endl;
        yarp::os::Time::delay(1);
        return -1;
    } else
        //LOG(INFO) << "readLabelsMapFile(): labels map loaded with " << labelsMap.size() << " label(s)" << std::endl;
        std::cout<<"Labels "<<LABELS<<" loaded correctly."<<std::endl;
        std::cout<<labelsMap.size()<<" labels have been loaded."<<std::endl;
        yarp::os::Time::delay(1);

    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Video source frames are going to be taken."<<std::endl;
    yarp::os::Time::delay(1);
    cv::Mat frame;
    tensorflow::Tensor tensor;
    std::vector<tensorflow::Tensor> outputs;
    double thresholdScore = 0.5;
    double thresholdIOU = 0.8;

    // Cuenta FPS
    int nFrames = 25;
    int iFrame = 0;
    double fps = 0.;
    time_t start, end;
    time(&start);
    tensorflow::TensorShape shape = tensorflow::TensorShape();
    shape.AddDim(1);
    shape.AddDim((tensorflow::int64)800);//cap.get(cv::CAP_PROP_FRAME_HEIGHT)->800
    shape.AddDim((tensorflow::int64)600);//cap.get(cv::CAP_PROP_FRAME_WIDTH)->600
    shape.AddDim(3);
    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Taking frames..."<<std::endl;
    yarp::os::Time::delay(1);
    inImg_i=inImg;

    while (true) {

        yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImgY = inImg_i->read();
        cv::Mat in_cv = cv::cvarrToMat((IplImage *)inImgY->getIplImage());
        frame=in_cv;

        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        std::cout << "Frame: " << iFrame << std::endl;

        if (nFrames % (iFrame + 1) == 0) {
            time(&end);
            fps = 1. * nFrames / difftime(end, start);
            time(&start);
        }
        iFrame++;

        // Pasar Mat a Tensor
        tensor = tensorflow::Tensor(tensorflow::DT_FLOAT, shape);
        tensorflow::Status readTensorStatus = readTensorFromMat(frame, tensor);
        if (!readTensorStatus.ok()) {
            //LOG(ERROR) << "Mat->Tensor conversion failed: " << readTensorStatus;
            std::cout<<std::endl;
            std::cout<<std::endl;
            std::cout<<"Mat OpenCV -> Tensor : FAIL"<<std::endl;
            yarp::os::Time::delay(1);
            return -1;
        }

        // Ejecutar graph
        outputs.clear();
        tensorflow::Status runStatus = session->Run({{inputLayer, tensor}}, outputLayer, {}, &outputs);
        if (!runStatus.ok()) {
            //LOG(ERROR) << "Running model failed: " << runStatus;
            std::cout<<std::endl;
            std::cout<<std::endl;
            std::cout<<"Running model status: FAIL"<<std::endl;
            yarp::os::Time::delay(1);
            return -1;
        }

        // Extraer resultados del vector
        tensorflow::TTypes<float>::Flat scores = outputs[1].flat<float>();
        tensorflow::TTypes<float>::Flat classes = outputs[2].flat<float>();
        tensorflow::TTypes<float>::Flat numDetections = outputs[3].flat<float>();
        tensorflow::TTypes<float, 3>::Tensor boxes = outputs[0].flat_outer_dims<float,3>();

        std::vector<size_t> goodIdxs = filterBoxes(scores, boxes, thresholdIOU, thresholdScore);
        for (size_t i = 0; i < goodIdxs.size(); i++){
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<"Detection: "<<labelsMap[classes(goodIdxs.at(i))]<< " -> Score: "<<scores(goodIdxs.at(i))<<std::endl;

        /* LOG(INFO) << "score:" << scores(goodIdxs.at(i)) << ",class:" << labelsMap[classes(goodIdxs.at(i))]
                      << " (" << classes(goodIdxs.at(i)) << "), box:" << "," << boxes(0, goodIdxs.at(i), 0) << ","
                      << boxes(0, goodIdxs.at(i), 1) << "," << boxes(0, goodIdxs.at(i), 2) << ","
                      << boxes(0, goodIdxs.at(i), 3);*/


        std::cout<<std::endl;

        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        drawBoundingBoxesOnImage(frame, scores, classes, boxes, labelsMap, goodIdxs);

        cv::putText(frame, std::to_string(fps).substr(0, 5), cv::Point(0, frame.rows), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255));
        //yarp_sender.send_post(frame, puerto_post);
        // A mano
        yarp::sig::ImageOf<yarp::sig::PixelBgr> C;
        C.setExternal(frame.data,frame.size[1],frame.size[0]);
        sender_port_post.write(C);
        cv::imshow("Video source: Processed", frame);
        cv::waitKey(5);
    }

}
    cv::destroyAllWindows();

    return 0;
}
