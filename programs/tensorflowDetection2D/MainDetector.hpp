#ifndef MAINDETECTOR_HPP
#define MAINDETECTOR_HPP

#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <time.h>
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
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/sig/Image.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.hpp>
#include "TensorflowDetector.hpp"


class maindetector
{
public:
int detect(std::string labels, std::string graph, yarp::os::Port sender_port_post, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *inImg);
private:
yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *inImg_i;
};
#endif //MAINDETECTOR_HPP
