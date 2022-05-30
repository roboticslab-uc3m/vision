#include "HogFaceDetector.hpp"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/cv/Cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(HFD, "rl.HogFaceDetector")
}

bool HogFaceDetector::open(yarp::os::Searchable& parameters)
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("HogFaceDetector");

    hogFaceDetector = dlib::get_frontal_face_detector();

    dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;

    return true;
}

bool HogFaceDetector::detect(const yarp::sig::Image & inYarpImg, yarp::os::Bottle & detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);
    cv::Mat inCvMatGray;

    cv::cvtColor(inCvMat, inCvMatGray, cv::COLOR_BGR2GRAY);

    dlib::cv_image<unsigned char> dlibImage(inCvMatGray);

    std::vector<dlib::rectangle> faceRects = hogFaceDetector(dlibImage);

    for (const auto & faceRect : faceRects)
    {
        cv::Rect object = cv::Rect(cv::Point2i(faceRect.left(), faceRect.top()), cv::Point2i(faceRect.right() + 1, faceRect.bottom() + 1));
        shape = predictor(dlibImage, faceRect);

        yarp::os::Property & dict = detectedObjects.addDict();
        dict.put("tlx", object.x);
        dict.put("tly", object.y);
        dict.put("brx", object.x + object.width);
        dict.put("bry", object.y + object.height);

        yarp::os::Value * list = yarp::os::Value::makeList();

        for (unsigned long j = 0; j < shape.num_parts(); j++)
        {
            list->asList()->addList() = {
                    yarp::os::Value(shape.part(j).x(), false),
                    yarp::os::Value(shape.part(j).y(), false)
            };
        }

        dict.put("landmarks", list);
    }

    return true;
}
